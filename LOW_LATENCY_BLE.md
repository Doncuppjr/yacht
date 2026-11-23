# Experimental latest-state HeadTracker BLE output

This experimental fork builds on ysoldak/HeadTracker (Unlicense).
It adds an advertising-restart fallback and notification credit control for
Nordic SoftDevice so only one notification may remain outstanding. Every
20 ms update encodes the latest position. Updates that cannot be transmitted
immediately are intentionally skipped, never queued behind obsolete positions.

## Source checkout layout

Place these two checkouts next to one another:

    workspace/
      ysoldak-HeadTracker/   # feature/latest-state-ble-credits
      tinygo-bluetooth-v013/ # feature/nordic-notification-complete

The HeadTracker go.mod temporarily replaces tinygo.org/x/bluetooth with the
sibling checkout. The Makefile resolves SoftDevice headers via go list -m.
Use TinyGo 0.40.1 and Go 1.25 (as verified on AiPal):

    make TARGET=xiao-ble VERSION=2.7.0-credit-experimental build

The resulting UF2 is under build/. Do NOT distribute the experimental firmware
as a stable release without functional and extended connection testing.

## Diagnostics

The USB serial log reports every ~10 s while connected:
- accepted: notification writes accepted by SoftDevice, includes bootstrap CRLF
- complete: SoftDevice completed-notification count
- wait: periodic ticks skipped because one notification is still outstanding
- full: unexpected NRF_ERROR_RESOURCES with credit gating

Counts reset on each new BLE connection except the cumulative full counter.
Do not equate a SoftDevice send completion with application-level receipt.

## Upstream strategy

Contribute the generic callback separately to tinygo-org/bluetooth.
Contribute the latest-state policy and advertising recovery to ysoldak/HeadTracker
as separate changes. When the TinyGo API is merged and released, remove the
local module replacement and pin the official release.

## Licensing

HeadTracker is Unlicense. The forked TinyGo Bluetooth library retains its
existing BSD-style license and notice requirements.

## Bench result (2026-11-23)

The original single-credit gate was too restrictive (~7.3 accepted notifications/s).
Window=4 successfully maintained ~37 accepted notifications/s in a 34-second
hardware test, with zero SoftDevice NRF_ERROR_RESOURCES responses, no C5
packet drops, and valid TX15 trainer input. Window=5 had ~39/s but many
queue-full responses (~9/s), and window=6 ~41/s but also queue-full responses.
The selected experimental window is four. Longer-term field testing is still
required. The BLE link's negotiated connection parameters should be measured.
