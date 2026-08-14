#include <FreeRTOS.h>
#include <task.h>
#include <string.h>

static constexpr configSTACK_DEPTH_TYPE kLoopStackWords = 2048u;

extern "C" BaseType_t __real_xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char *const pcName,
    const configSTACK_DEPTH_TYPE usStackDepth,
    void *const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t *const pxCreatedTask);

extern "C" BaseType_t __wrap_xTaskCreate(
    TaskFunction_t pxTaskCode,
    const char *const pcName,
    const configSTACK_DEPTH_TYPE usStackDepth,
    void *const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t *const pxCreatedTask)
{
    configSTACK_DEPTH_TYPE depth = usStackDepth;

    if (pcName != nullptr &&
        strcmp(pcName, "loop") == 0 &&
        depth < kLoopStackWords) {
        depth = kLoopStackWords;
    }

    return __real_xTaskCreate(
        pxTaskCode,
        pcName,
        depth,
        pvParameters,
        uxPriority,
        pxCreatedTask);
}
