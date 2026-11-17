#pragma once
// Process stop ownership. No Python, camera or motor SDK initialization.
void request_app_stop();
bool app_stop_requested();
void install_app_sigint_handler();
void mark_app_cleanup_started();

