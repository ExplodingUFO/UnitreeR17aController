//! RAII protection against idle display/system sleep while a device is connected.
//! Called by the existing desktop state observer, never by arm-control. It does
//! not declare user activity or change lock, sleep or Energy Saver settings.
#[derive(Default)]
pub struct IdleSleep {
    #[cfg(target_os = "macos")]
    protection: macos::Protection<macos::Native>,
}
impl IdleSleep {
    pub fn update(&mut self, connected: bool) -> Result<(), String> {
        #[cfg(target_os = "macos")]
        {
            self.protection.update(connected, std::time::Instant::now())
        }
        #[cfg(not(target_os = "macos"))]
        {
            let _ = connected;
            Ok(())
        }
    }
}

#[cfg(target_os = "macos")]
mod macos {
    use std::{
        ffi::c_void,
        time::{Duration, Instant},
    };
    const TYPES: [&str; 2] = ["PreventUserIdleSystemSleep", "PreventUserIdleDisplaySleep"];
    const NAME: &str = "R1-7a Controller: connected device";
    const RETRY: Duration = Duration::from_secs(5);

    pub(super) trait Assertions {
        fn create(&mut self, kind: &'static str) -> Result<u32, String>;
        fn release(&mut self, id: u32) -> Result<(), String>;
    }
    #[derive(Default)]
    pub(super) struct Native;
    impl Assertions for Native {
        fn create(&mut self, kind: &'static str) -> Result<u32, String> {
            create(kind, NAME)
        }
        fn release(&mut self, id: u32) -> Result<(), String> {
            // SAFETY: tokens belong to this process and are removed on success.
            os_result("IOPMAssertionRelease", unsafe { IOPMAssertionRelease(id) })
        }
    }

    /// Only integer OS tokens persist across await; CF objects are call-local.
    #[derive(Default)]
    pub(super) struct Protection<B: Assertions> {
        backend: B,
        ids: [Option<u32>; 2],
        connected: bool,
        retry_at: Option<Instant>,
        last_error: Option<String>,
    }
    impl<B: Assertions> Protection<B> {
        pub(super) fn update(&mut self, connected: bool, now: Instant) -> Result<(), String> {
            if self.connected == connected && self.retry_at.is_none_or(|at| now < at) {
                return Ok(());
            }
            self.connected = connected;
            let mut errors = Vec::new();
            for (kind, id) in TYPES.into_iter().zip(&mut self.ids) {
                let result = match (connected, *id) {
                    (true, None) => self.backend.create(kind).map(|created| *id = Some(created)),
                    (false, Some(current)) => self.backend.release(current).map(|()| *id = None),
                    _ => Ok(()),
                };
                if let Err(error) = result {
                    errors.push(format!("{kind}: {error}"));
                }
            }
            if errors.is_empty() {
                self.retry_at = None;
                self.last_error = None;
                Ok(())
            } else {
                self.retry_at = Some(now + RETRY);
                let error = errors.join("; ");
                if self.last_error.as_ref() == Some(&error) {
                    return Ok(());
                }
                self.last_error = Some(error.clone());
                Err(error)
            }
        }
    }
    impl<B: Assertions> Drop for Protection<B> {
        fn drop(&mut self) {
            for id in self.ids.iter_mut().filter_map(Option::take) {
                let _ = self.backend.release(id);
            }
        }
    }

    type CFRef = *const c_void;
    #[link(name = "IOKit", kind = "framework")]
    unsafe extern "C" {
        fn IOPMAssertionCreateWithName(kind: CFRef, level: u32, name: CFRef, id: *mut u32) -> i32;
        fn IOPMAssertionRelease(id: u32) -> i32;
    }
    #[link(name = "CoreFoundation", kind = "framework")]
    unsafe extern "C" {
        fn CFStringCreateWithBytes(
            allocator: CFRef,
            bytes: *const u8,
            length: isize,
            encoding: u32,
            external: u8,
        ) -> CFRef;
        fn CFRelease(value: CFRef);
    }
    struct CFString(CFRef);
    impl CFString {
        fn new(value: &str) -> Result<Self, String> {
            // SAFETY: valid UTF-8 bytes/length for a synchronous call; null
            // selects the default allocator, 0x08000100 is UTF-8 encoding.
            let pointer = unsafe {
                CFStringCreateWithBytes(
                    std::ptr::null(),
                    value.as_ptr(),
                    value.len() as isize,
                    0x0800_0100,
                    0,
                )
            };
            if pointer.is_null() {
                Err("CFStringCreateWithBytes failed".into())
            } else {
                Ok(Self(pointer))
            }
        }
    }
    impl Drop for CFString {
        fn drop(&mut self) {
            // SAFETY: one owned non-null Create-rule CF reference.
            unsafe { CFRelease(self.0) };
        }
    }
    fn os_result(operation: &str, code: i32) -> Result<(), String> {
        if code == 0 {
            Ok(())
        } else {
            Err(format!("{operation}: 0x{:08x}", code as u32))
        }
    }
    fn create(kind: &str, name: &str) -> Result<u32, String> {
        let kind = CFString::new(kind)?;
        let name = CFString::new(name)?;
        let mut id = 0;
        // SAFETY: strings/output storage outlive the call. 255 is
        // kIOPMAssertionLevelOn; only the two idle-sleep assertion types are used.
        let result = unsafe { IOPMAssertionCreateWithName(kind.0, 255, name.0, &mut id) };
        os_result("IOPMAssertionCreateWithName", result)?;
        Ok(id)
    }

    #[cfg(test)]
    mod tests {
        use super::super::IdleSleep;
        use super::*;
        use std::{
            collections::BTreeSet,
            sync::{Arc, Mutex},
            time::{Duration, Instant},
        };

        #[derive(Default)]
        struct Calls {
            next: u32,
            create: [u32; 2],
            release: u32,
            held: BTreeSet<u32>,
            fail_create: Option<&'static str>,
            fail_release: bool,
        }
        #[derive(Clone, Default)]
        struct Fake(Arc<Mutex<Calls>>);
        impl Assertions for Fake {
            fn create(&mut self, kind: &'static str) -> Result<u32, String> {
                let mut calls = self.0.lock().unwrap();
                calls.create[usize::from(kind == TYPES[1])] += 1;
                if calls.fail_create == Some(kind) {
                    return Err("injected create error".into());
                }
                calls.next += 1;
                let id = calls.next;
                calls.held.insert(id);
                Ok(id)
            }
            fn release(&mut self, id: u32) -> Result<(), String> {
                let mut calls = self.0.lock().unwrap();
                calls.release += 1;
                if calls.fail_release {
                    return Err("injected release error".into());
                }
                assert!(calls.held.remove(&id));
                Ok(())
            }
        }

        #[test]
        fn connection_scope_retains_tokens_across_updates_and_releases_on_disconnect_and_drop() {
            let backend = Fake::default();
            let now = Instant::now();
            let mut protection = Protection {
                backend: backend.clone(),
                ids: [None; 2],
                connected: false,
                retry_at: None,
                last_error: None,
            };
            protection.update(false, now).unwrap();
            assert_eq!(backend.0.lock().unwrap().create, [0, 0]);
            protection.update(true, now).unwrap();
            for i in 1..1000 {
                protection
                    .update(true, now + Duration::from_millis(i))
                    .unwrap();
            }
            assert_eq!(backend.0.lock().unwrap().create, [1, 1]);
            assert_eq!(backend.0.lock().unwrap().held.len(), 2);
            protection.update(false, now).unwrap();
            assert!(backend.0.lock().unwrap().held.is_empty());
            protection.update(true, now).unwrap();
            assert_eq!(backend.0.lock().unwrap().create, [2, 2]);
            drop(protection);
            let calls = backend.0.lock().unwrap();
            assert_eq!(calls.release, 4);
            assert!(calls.held.is_empty());
        }

        #[test]
        fn native_failures_keep_partial_protection_retry_missing_tokens_and_do_not_spam_errors() {
            let backend = Fake::default();
            backend.0.lock().unwrap().fail_create = Some(TYPES[0]);
            let mut protection = Protection {
                backend: backend.clone(),
                ids: [None; 2],
                connected: false,
                retry_at: None,
                last_error: None,
            };
            let now = Instant::now();
            assert!(
                protection
                    .update(true, now)
                    .unwrap_err()
                    .contains("injected create error")
            );
            assert_eq!(backend.0.lock().unwrap().held.len(), 1);
            protection.update(true, now + RETRY / 2).unwrap();
            assert_eq!(backend.0.lock().unwrap().create, [1, 1]);
            protection.update(true, now + RETRY).unwrap(); // Same error is already reported.
            assert_eq!(backend.0.lock().unwrap().create, [2, 1]);
            backend.0.lock().unwrap().fail_create = None;
            protection.update(true, now + RETRY * 2).unwrap();
            assert_eq!(backend.0.lock().unwrap().held.len(), 2);
            assert_eq!(backend.0.lock().unwrap().create, [3, 1]);
            backend.0.lock().unwrap().fail_release = true;
            assert!(
                protection
                    .update(false, now + RETRY * 3)
                    .unwrap_err()
                    .contains("injected release error")
            );
            assert_eq!(backend.0.lock().unwrap().held.len(), 2);
            backend.0.lock().unwrap().fail_release = false;
            protection.update(false, now + RETRY * 4).unwrap();
            assert!(backend.0.lock().unwrap().held.is_empty());
            protection.update(true, now + RETRY * 5).unwrap();
            drop(protection);
            assert!(backend.0.lock().unwrap().held.is_empty());
        }

        #[test]
        fn exit_cleanup_attempts_remaining_tokens_even_when_release_previously_failed() {
            let backend = Fake::default();
            let mut protection = Protection {
                backend: backend.clone(),
                ids: [None; 2],
                connected: false,
                retry_at: None,
                last_error: None,
            };
            let now = Instant::now();
            protection.update(true, now).unwrap();
            backend.0.lock().unwrap().fail_release = true;
            assert!(protection.update(false, now).is_err());
            backend.0.lock().unwrap().fail_release = false;
            drop(protection);
            assert!(backend.0.lock().unwrap().held.is_empty());
            assert_eq!(backend.0.lock().unwrap().release, 4);
        }

        #[test]
        fn assertion_guard_is_send_without_retaining_cf_pointers() {
            fn is_send<T: Send>() {}
            is_send::<IdleSleep>();
        }

        // These opt-in tests touch only this test process's assertion IDs. They do
        // not launch Tauri, open a port, change settings, sleep or lock the machine.
        #[link(name = "IOKit", kind = "framework")]
        unsafe extern "C" {
            fn IOPMAssertionCopyProperties(id: u32) -> CFRef;
        }
        #[link(name = "CoreFoundation", kind = "framework")]
        unsafe extern "C" {
            fn CFDictionaryGetValue(dictionary: CFRef, key: CFRef) -> CFRef;
            fn CFEqual(first: CFRef, second: CFRef) -> u8;
            fn CFNumberGetValue(number: CFRef, kind: isize, value: *mut i32) -> u8;
        }
        struct Properties(CFRef);
        impl Drop for Properties {
            fn drop(&mut self) {
                // SAFETY: owned non-null Copy-rule CF dictionary.
                unsafe { CFRelease(self.0) };
            }
        }
        fn verify_native(id: u32, kind: &str) {
            // SAFETY: id was created by this test process.
            let pointer = unsafe { IOPMAssertionCopyProperties(id) };
            assert!(!pointer.is_null(), "own assertion {id} missing");
            let properties = Properties(pointer);
            for (key, expected) in [("AssertType", kind), ("AssertName", NAME)] {
                let key = CFString::new(key).unwrap();
                let expected = CFString::new(expected).unwrap();
                // SAFETY: dictionary and string keys remain alive; value is borrowed.
                let value = unsafe { CFDictionaryGetValue(properties.0, key.0) };
                assert!(!value.is_null());
                assert_ne!(unsafe { CFEqual(value, expected.0) }, 0);
            }
            let key = CFString::new("AssertLevel").unwrap();
            let number = unsafe { CFDictionaryGetValue(properties.0, key.0) };
            assert!(!number.is_null());
            let mut level = 0;
            // kCFNumberSInt32Type = 3, with correctly sized writable storage.
            assert_ne!(unsafe { CFNumberGetValue(number, 3, &mut level) }, 0);
            assert_eq!(level, 255);
            println!(
                "pid={} assertion={id} type={kind} level={level}",
                std::process::id()
            );
        }
        fn verify_released(ids: &[Option<u32>; 2]) {
            for id in ids.iter().flatten() {
                let dictionary = unsafe { IOPMAssertionCopyProperties(*id) };
                if !dictionary.is_null() {
                    unsafe { CFRelease(dictionary) };
                }
                assert!(dictionary.is_null(), "own assertion {id} was not released");
            }
        }

        #[test]
        #[ignore = "requires the local macOS power service; owns only this process's assertions"]
        fn native_assertions_exist_release_reconnect_and_drop() {
            let mut protection = IdleSleep::default();
            protection.update(false).unwrap();
            protection.update(true).unwrap();
            let first = protection.protection.ids;
            for (id, kind) in first.into_iter().zip(TYPES) {
                verify_native(id.unwrap(), kind);
            }
            protection.update(true).unwrap();
            assert_eq!(first, protection.protection.ids);
            protection.update(false).unwrap();
            verify_released(&first);
            protection.update(true).unwrap();
            let second = protection.protection.ids;
            for (id, kind) in second.into_iter().zip(TYPES) {
                verify_native(id.unwrap(), kind);
            }
            drop(protection);
            verify_released(&second);
            println!(
                "all assertions created by pid={} released",
                std::process::id()
            );
        }

        #[test]
        #[ignore = "requires the local macOS power service; invalid type creates no assertion"]
        fn native_errors_are_returned_and_owned_tokens_still_release() {
            let error = create(
                "R1-7a unsupported assertion test",
                "R1-7a assertion error test",
            )
            .unwrap_err();
            assert!(error.starts_with("IOPMAssertionCreateWithName: 0x"));
            println!("expected OS failure: {error}");
            let mut native = Native;
            let id = native.create(TYPES[0]).unwrap();
            verify_native(id, TYPES[0]);
            native.release(id).unwrap();
            verify_released(&[Some(id), None]);
            assert!(
                native
                    .release(id)
                    .unwrap_err()
                    .starts_with("IOPMAssertionRelease: 0x")
            );
        }
    }
}
