# Official sample calibration and protocol oracle

Source: `unitreerobotics/r1-7a_sdk`, commit
`0803d033b530fcfa0e6fa9236de68c2773fc9402`.
`joint_calib_{right,left}.txt` are byte-for-byte copies of `config/` files.
They are SDK samples; the actual arm's zero positions have not been verified.
`protocol.rs` embeds these files at compile time.

The SDK's original BSD-3-Clause copyright and license are retained in
`UNITREE-LICENSE`. See the repository's `THIRD_PARTY_NOTICES.md` for attribution.

`protocol-golden.json` was generated on macOS arm64 by compiling the original
`thirdparty/motor_controller/src/MotorProtocol.cpp`, with the command in
`generate_protocol_golden.cpp`. Run it from this directory and set `SDK` to the
fixed checkout. `sdk_compile_shim.h` only resolves the macOS `M_PI` macro name
collision; it does not alter SDK formulas, CRC, or packing code.

The oracle covers all three reductions, positive/negative encodable float32
boundaries, zero, disabled, clear-fault, and full signed feedback integer ranges.
The SDK narrows before clamping: its advertised ranges can overflow for some
reductions. Rust rejects those inputs instead of reproducing a wrapped command.

No physical port was opened for these tests. Input is purged before each
exchange and previous frames are never reused. The wire protocol has no sequence
number, so a same-ID old reply arriving after a new transmit cannot be identified
as old. Requested finite OS I/O timeouts and a host deadline cannot preempt a
faulty native driver; late reads are rejected, with no automatic command resend.
