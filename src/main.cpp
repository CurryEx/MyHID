#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include <vector>
#include <functional>
#include "driver/uart.h"
#include <cmath>
#include <string>

typedef enum {
    ORIGINAL = 0,
    COMPRESSED = 1,
} SerialDataMode;

typedef enum {
    BUTTON_MODE_TOGGLE = 0,
    BUTTON_MODE_CONTINUOUS = 1,
} ButtonMode;

typedef struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
} QueueEvent;

typedef struct {
    uint8_t pressed;
    uint8_t touchId;
    unsigned short x;
    unsigned short y;
} TouchPoint;

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    char data;
    std::vector<Point>* points = new std::vector<Point>();
} Shape;

static const char *TAG = "MyHID";
static const int TOUCH_POINT_DATA_LENGTH = 6;

QueueHandle_t appEventQueue = NULL;
std::vector<Shape>* shapes = new std::vector<Shape>();
TouchPoint touchPoints[10];
SerialDataMode serialDataMode = ORIGINAL;
ButtonMode buttonMode = BUTTON_MODE_CONTINUOUS;
int compressSplitCount = 1;
char dataPrefix = '\x00';
char dataSuffix = '\xff';
char* buttonsData = NULL;
int buttonsDataLength = 0;
std::function<void(char*, int)> serialReceiveCallback;
volatile bool sendButtonData = false;
char* responseData = NULL;
char serial0RxBuffer[256];
int sendDelayMS = 10;
int hostDataLength = 6;

Shape* hitCheck(TouchPoint touchPoint);
bool isPointInShape(TouchPoint touchPoint, Shape& shape);
static void hidHostGenericReportCallback(const uint8_t *const data, const int length);
void hidHostInterfaceCallback(hid_host_device_handle_t hidDeviceHandle, const hid_host_interface_event_t event, void *arg);
void hidHostDeviceEvent(hid_host_device_handle_t hidDeviceHandle, const hid_host_driver_event_t event, void *arg);
static void usbLibTask(void *arg);
void hidHostDeviceCallback(hid_host_device_handle_t hidDeviceHandle, const hid_host_driver_event_t event, void *arg);
void serial0Send(const char* data, size_t length);
void printHex(char* data, size_t length);
static void serialWriteTask(void *arg);
static void serialReadTask(void *arg);
static void customBehavior();

// 自定义MyHID的各种行为
static void customBehavior()
{
    // 触摸按钮定义



    // ========================================

    // 发给主机的数据包自定义
    // 数据包模式 
    // ORIGINAL 原始按下了哪些按钮(给人看的) 
    // 输出示例[前缀][按钮00][按钮01][按钮02][后缀]
    // 若按钮按下则对应字节有值 否则为00
    // COMPRESSED 原始传输方式需要较大的数据量 可以想办法进行压缩 不难得出 使用串口传输时 一个字节 FF 可以代表最多八个按钮
    // 0000 0000 所以可以将m个按钮(最多八个)成为一组 假设有n个按钮 则需要n//m组数据 m取值范围[1, 8]
    serialDataMode = COMPRESSED;
    // 压缩分组数量 m
    compressSplitCount = 5;
    // 数据包前缀
    dataPrefix = '\x28';
    // 数据包后缀
    dataSuffix = '\x29';
    // 数据包发送延迟 在9600 8n1传输模式下 一秒最多发送1065字节 结合数据包计算
    sendDelayMS = 10;
    // 主机与设备通信数据包长度
    hostDataLength = 6;
    // 按钮模式
    // BUTTON_MODE_TOGGLE 按下一次发送一次数据
    // BUTTON_MODE_CONTINUOUS 持续发送当前按钮状态数据
    buttonMode = BUTTON_MODE_CONTINUOUS;

    // 自定义如何处理主机发来的数据 以下是一个示例
    serialReceiveCallback = [](char* data, int length) {
        // 我定义主机发来的数据包以11开头 22结尾 第二个字节为控制模式 后面跟4个字节的数据
        // 我的控制模式 72~81是其他待实现的功能 82 是停止 83是开始发送数据
        // 例如 11 83 00 00 00 00 22 表示开始发送数据
        int controlMode = static_cast<int>(data[1]);
        // 停止 休眠
        if (controlMode == 82 || controlMode == 72) 
        {
            sendButtonData = false;
        }
        // 身份验证 暂时没有实现 简单将数据原样返回
        else if (controlMode == 76) 
        {
            for (int i = 1; i < 5; ++i) 
            {
                responseData[i] = data[i];
            }
            serial0Send(responseData, 6);
        } 
        // 开始发送数据
        else if (controlMode == 83) 
        {
            sendButtonData = true;
        }
    };
}

// 自定义结束 下面是主要代码

Shape* hitCheck(TouchPoint touchPoint) 
{
    for (Shape& shape : *shapes) 
    {
        if (isPointInShape(touchPoint, shape)) 
        {
            return &shape;
        }
    }
    return nullptr;
}

bool isPointInShape(TouchPoint touchPoint, Shape& shape) 
{
    int n = shape.points->size();
    if (n < 3) return false; // 多边形至少需要3个点

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) 
    {
        Point& pi = shape.points->at(i);
        Point& pj = shape.points->at(j);
        if (((pi.y > touchPoint.y) != (pj.y > touchPoint.y)) &&
            (touchPoint.x < (pj.x - pi.x) * (touchPoint.y - pi.y) / (pj.y - pi.y) + pi.x)) 
        {
            inside = !inside;
        }
    }
    return inside;
}

static void hidHostGenericReportCallback(const uint8_t *const data, const int length)
{
    // printf("data length: %d\r\ndata:\r\n", length);   
    // printHex((char*)data, length);

    //数据示例10    00       01      4E72        D93A         ...00 00 0000 0000 
    //       起始位 是否按下 触摸id   x坐标小端序  y坐标小端序   ...下一组共十组

    // 确保数据长度足够
    if (length != 64) 
    {
        printf("data error\n");
        return;
    }

    buttonsData[buttonsDataLength - 1] = dataSuffix;
    buttonsData[0] = dataPrefix;
    for (int i = 1; i < buttonsDataLength - 1; i++) 
    {
        buttonsData[i] = 0;
    }

    for (int i = 0; i < 10; i++) 
    {
        int offset = i * TOUCH_POINT_DATA_LENGTH + 1;
        // 反正每次触摸事件都会清空数组 所以不需要判断触摸点id 只需要判断是否按下即可
        if(data[offset] == 0)
        {
            continue;
        }
        touchPoints[i].pressed = data[offset];
        touchPoints[i].touchId = data[offset + 1];
        touchPoints[i].x = data[offset + 2] | (data[offset + 3] << 8);
        touchPoints[i].y = data[offset + 4] | (data[offset + 5] << 8);

        // printf("touch: %d press: %d id: %d x: %d y: %d\r\n", i, touchPoints[i].pressed, touchPoints[i].touchId, touchPoints[i].x, touchPoints[i].y);
        Shape* target = hitCheck(touchPoints[i]);
        if (target) 
        {
            if(serialDataMode == COMPRESSED)
            {
                buttonsData[static_cast<int>(target->data) / compressSplitCount + 1]
                 |= 1 << (static_cast<int>(target->data) % compressSplitCount);
            }
            else
            {
                buttonsData[static_cast<int>(target->data) + 1] = static_cast<int>(target->data) + 1;
            }
        }
    }

    if(buttonMode == BUTTON_MODE_TOGGLE && sendButtonData)
    {
        serial0Send(buttonsData, buttonsDataLength);
    }
}

void hidHostInterfaceCallback(hid_host_device_handle_t hidDeviceHandle,
                                 const hid_host_interface_event_t event,
                                 void *arg)
{
    uint8_t data[64] = { 0 };
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hidDeviceHandle, &dev_params));

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hidDeviceHandle,
                                                                  data,
                                                                  64,
                                                                  &data_length));

        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) 
        {
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto) 
            {
                printf("keyboard is not supported\n");
            } 
            else if (HID_PROTOCOL_MOUSE == dev_params.proto) 
            {
                printf("mouse is not supported\n");
            }
        } 
        else 
        {
            hidHostGenericReportCallback(data, data_length);
        }

        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device '%d' DISCONNECTED", dev_params.iface_num);
        ESP_ERROR_CHECK(hid_host_device_close(hidDeviceHandle));
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGE(TAG, "HID Device '%d' TRANSFER ERROR", dev_params.iface_num);
        break;

    default:
        ESP_LOGE(TAG, "HID Device '%d' Unhandled event", dev_params.iface_num);
        break;
    }
}

// 处理hid事件
void hidHostDeviceEvent(hid_host_device_handle_t hidDeviceHandle,
                           const hid_host_driver_event_t event,
                           void *arg)
{
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hidDeviceHandle, &dev_params));

    switch (event)
    {
        // 如果是HID设备连接
        case HID_HOST_DRIVER_EVENT_CONNECTED:
        {
            ESP_LOGI(TAG, "HID Device, protocol '%d' CONNECTED", dev_params.proto);

            // 注册设备及回调函数
            const hid_host_device_config_t dev_config = {
                .callback = hidHostInterfaceCallback,
                .callback_arg = NULL
            };

            ESP_ERROR_CHECK(hid_host_device_open(hidDeviceHandle, &dev_config));
            if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) 
            {
                ESP_ERROR_CHECK(hid_class_request_set_protocol(hidDeviceHandle, HID_REPORT_PROTOCOL_BOOT));
                if (HID_PROTOCOL_KEYBOARD == dev_params.proto) 
                {
                    ESP_ERROR_CHECK(hid_class_request_set_idle(hidDeviceHandle, 0, 0));
                }
            } 
            else 
            {
                // 输出当前设备的sub_class proto
                ESP_LOGI(TAG, "sub_class: %d proto: %d\r\n", dev_params.sub_class, dev_params.proto);
                // 发送指定的USB报文
                uint8_t report_data[] = {0x07, 0x02, 0x00};
                ESP_LOGI(TAG, "hid_class_request_set_report");
                ESP_ERROR_CHECK(hid_class_request_set_report(hidDeviceHandle, 0x03, 0x07, report_data, sizeof(report_data)));
            }
            ESP_LOGI(TAG, "hid_host_device_start");
            ESP_ERROR_CHECK(hid_host_device_start(hidDeviceHandle));
            break;
        }
        default:
            break;
    }
}

static void usbLibTask(void *arg)
{
    while (true) 
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) 
        {
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }

    ESP_LOGI(TAG, "USB shutdown");
    vTaskDelay(sendDelayMS / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

void hidHostDeviceCallback(hid_host_device_handle_t hidDeviceHandle,
                              const hid_host_driver_event_t event,
                              void *arg)
{
    ESP_LOGI(TAG, "HID Device event");
    // 收到hid事件之后就推到事件队列
    QueueEvent newEvent = {
        .handle = hidDeviceHandle,
        .event = event,
        .arg = arg
    };

    if (appEventQueue) 
    {
        xQueueSend(appEventQueue, &newEvent, 0);
    }
}

extern "C" void app_main(void)
{
    customBehavior();
    // 返回报文
    responseData = new char[6]{0};
    responseData[0] = dataPrefix;
    responseData[5] = dataSuffix;
    // 串口初始化
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, GPIO_NUM_43, GPIO_NUM_44, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    // 初始化按钮数据组
    if(serialDataMode == COMPRESSED)
    {
        buttonsDataLength = static_cast<int>(ceil(1.0 * shapes->size() / compressSplitCount)) + 2;
    }
    else
    {
        buttonsDataLength = shapes->size() + 2;
    }
    buttonsData = new char[buttonsDataLength];
    buttonsData[buttonsDataLength - 1] = dataSuffix;
    buttonsData[0] = dataPrefix;
    for (int i = 1; i < buttonsDataLength - 1; i++) 
    {
        buttonsData[i] = 0;
    }
    printf("button count: %d data length: %d\r\n", shapes->size(), buttonsDataLength);

    // 初始化usb库
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = NULL
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // 初始化usb库 开一个task处理断连
    BaseType_t task_created;
    task_created = xTaskCreatePinnedToCore(usbLibTask,
                                           "usb_events",
                                           4096,
                                           xTaskGetCurrentTaskHandle(),
                                           2,
                                           NULL, 
                                           0);
    assert(task_created == pdTRUE);
    ESP_LOGI(TAG, "usb lib task created");

    // 初始化串口发送task
    task_created = xTaskCreatePinnedToCore(serialWriteTask,
                                        "serial_write",
                                        4096,
                                        xTaskGetCurrentTaskHandle(),
                                        2,
                                        NULL, 
                                        0);
    assert(task_created == pdTRUE);
    ESP_LOGI(TAG, "serial write task created");

    // 初始化串口接收task
    task_created = xTaskCreatePinnedToCore(serialReadTask,
                                        "serial_read",
                                        4096,
                                        xTaskGetCurrentTaskHandle(),
                                        2,
                                        NULL, 
                                        0);
    assert(task_created == pdTRUE);
    ESP_LOGI(TAG, "serial read task created");

    // 创建事件队列用于在hid接收到事件之后在别的task中处理
    appEventQueue = xQueueCreate(10, sizeof(QueueEvent));
    QueueEvent event;

    // 初始化hid库
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 1,
        // 收到了hid事件后调用这个
        .callback = hidHostDeviceCallback,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    ESP_LOGI(TAG, "MyHID started");
    while (1)
    {
        if (xQueueReceive(appEventQueue, &event, portMAX_DELAY)) 
        {
            ESP_LOGI(TAG, "xQueueReceive");
            hidHostDeviceEvent(event.handle,
                                event.event,
                                event.arg);
        }
    }
}

void serial0Send(const char* data, size_t length) 
{
    uart_write_bytes(UART_NUM_0, data, length);
}

void printHex(char* data, size_t length) 
{
    for (size_t i = 0; i < length; i++) 
    {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}

static void serialWriteTask(void *arg)
{
    while (true)
    {
        if(sendButtonData && buttonMode == BUTTON_MODE_CONTINUOUS)
        {
            // printHex(buttonsData, buttonsDataLength);
            serial0Send(buttonsData, buttonsDataLength);
        }
        else
        {

        }

        // 以1000hz发送
        // vTaskDelay(1 / portTICK_PERIOD_MS);
        // 以0.01秒一次发送
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void serialReadTask(void *arg)
{
    while (true)
    {
        int len = uart_read_bytes(UART_NUM_0, serial0RxBuffer, hostDataLength, 100);
        if (len > 0) 
        {
            serial0RxBuffer[len] = '\0';
            printf("read! len: %d data: ", len);
            printHex(serial0RxBuffer, len);
            if (len != hostDataLength) 
            {
                printf("data length error\n");
                continue;
            }
            serialReceiveCallback(serial0RxBuffer, len);
        }
    }
}