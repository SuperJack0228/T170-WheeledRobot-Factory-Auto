#pragma once
// Read-only cancellation query. Ownership and writes remain in the hardware adapter.
bool hardware_abort_requested();

bool app_stop_requested();
void hardware_abort_sleep_ms(int ms);
