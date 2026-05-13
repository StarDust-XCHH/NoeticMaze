#include "debug_report.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#define DEBUG_EVENT_QUEUE_LEN 8U

static DebugEvent_t s_debug_events[DEBUG_EVENT_QUEUE_LEN];
static uint8_t s_debug_head = 0U;
static uint8_t s_debug_tail = 0U;
static uint8_t s_debug_count = 0U;
static uint32_t s_debug_drop_count = 0U;

void Debug_Post(uint8_t module,
                uint8_t stage,
                int16_t code,
                float a,
                float b,
                float c,
                float d)
{
    DebugEvent_t event;

    event.tick = osKernelGetTickCount();
    event.module = module;
    event.stage = stage;
    event.code = code;
    event.a = a;
    event.b = b;
    event.c = c;
    event.d = d;

    taskENTER_CRITICAL();

    if (s_debug_count >= DEBUG_EVENT_QUEUE_LEN) {
        s_debug_tail = (uint8_t)((s_debug_tail + 1U) % DEBUG_EVENT_QUEUE_LEN);
        s_debug_count--;
        s_debug_drop_count++;
    }

    s_debug_events[s_debug_head] = event;
    s_debug_head = (uint8_t)((s_debug_head + 1U) % DEBUG_EVENT_QUEUE_LEN);
    s_debug_count++;

    taskEXIT_CRITICAL();
}

bool Debug_TryPop(DebugEvent_t *out_event)
{
    if (out_event == NULL) {
        return false;
    }

    taskENTER_CRITICAL();

    if (s_debug_count == 0U) {
        taskEXIT_CRITICAL();
        return false;
    }

    *out_event = s_debug_events[s_debug_tail];
    s_debug_tail = (uint8_t)((s_debug_tail + 1U) % DEBUG_EVENT_QUEUE_LEN);
    s_debug_count--;

    taskEXIT_CRITICAL();
    return true;
}

uint32_t Debug_GetDropCount(void)
{
    uint32_t drop_count;

    taskENTER_CRITICAL();
    drop_count = s_debug_drop_count;
    taskEXIT_CRITICAL();

    return drop_count;
}
