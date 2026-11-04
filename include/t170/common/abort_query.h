#pragma once
// Read-only cancellation query. Ownership and writes remain in the hardware adapter.
bool hardware_abort_requested();
