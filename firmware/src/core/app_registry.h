// app_registry.h
// Shared registry of the active app's input/output interfaces.
//
// Each platform's main.c calls app_get_*_interfaces() to discover the
// interfaces an app has registered, then publishes them here so shared
// code (router, CDC commands, etc.) can introspect what the app exposes
// without hard-coding per-app knowledge.

#ifndef APP_REGISTRY_H
#define APP_REGISTRY_H

#include <stdint.h>
#include "core/input_interface.h"
#include "core/output_interface.h"

// Register the active interface arrays. Pointers are cached as-is — the
// caller owns the storage and must keep it valid for the lifetime of the
// program (the typical pattern: static const arrays in app.c).
void app_registry_set(const InputInterface* const* inputs, uint8_t input_count,
                      const OutputInterface* const* outputs, uint8_t output_count);

// Retrieve registered inputs. *count = 0 and returns NULL if not set.
const InputInterface* const* app_registry_inputs(uint8_t* count);

// Retrieve registered outputs. *count = 0 and returns NULL if not set.
const OutputInterface* const* app_registry_outputs(uint8_t* count);

// Stable string names for router enums (for web/JSON consumers).
// Returns a static lower-case identifier; never NULL.
const char* app_registry_input_source_name(input_source_t source);
const char* app_registry_output_target_name(output_target_t target);
const char* app_registry_routing_mode_name(uint8_t mode);
const char* app_registry_merge_mode_name(uint8_t mode);

#endif // APP_REGISTRY_H
