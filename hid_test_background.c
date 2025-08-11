#include "btstack_config.h"

#include "pico/cyw43_arch.h"
#include "pico/btstack_chipset_cyw43.h"
#include "pico/stdlib.h"
#include "pico/rand.h"
#include "btstack.h"
#include "btstack_run_loop.h"

#include <stdint.h>
#include <string.h>

static const char hid_device_name[] = "Wireless Gamepad";
static const char service_name[] = "Wireless Gamepad";
static uint8_t hid_service_buffer[700] = {0};
static uint8_t pnp_service_buffer[700] = {0};
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint16_t hid_cid = 0;

//uint8_t bd_addr[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xA0};

#define DESCRIPTOR_LEN 139
const uint8_t sinput_hid_report_descriptor[DESCRIPTOR_LEN] = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)

    // INPUT REPORT ID 0x01 - Main gamepad data
    0x85, 0x01, //   Report ID (1)

    // Padding bytes (bytes 2-3) - Plug status and Charge Percent (0-100)
    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined)
    0x09, 0x01,       //   Usage (Vendor Usage 1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0xFF,       //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // --- 32 buttons ---
    0x05, 0x09, // Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (Button 1)
    0x29, 0x20, //   Usage Maximum (Button 32)
    0x15, 0x00, //   Logical Min (0)
    0x25, 0x01, //   Logical Max (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x20, //   Report Count (32)
    0x81, 0x02, //   Input (Data,Var,Abs)

    // Analog Sticks and Triggers
    0x05, 0x01, // Usage Page (Generic Desktop)
    // Left Stick X (bytes 8-9)
    0x09, 0x30, //   Usage (X)
    // Left Stick Y (bytes 10-11)
    0x09, 0x31, //   Usage (Y)
    // Right Stick X (bytes 12-13)
    0x09, 0x32, //   Usage (Z)
    // Right Stick Y (bytes 14-15)
    0x09, 0x35, //   Usage (Rz)
    // Right Trigger (bytes 18-19)
    0x09, 0x33, //   Usage (Rx)
    // Left Trigger  (bytes 16-17)
    0x09, 0x34,       //  Usage (Ry)
    0x16, 0x00, 0x80, //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, //   Logical Maximum (32767)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x06,       //   Report Count (6)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Padding/Reserved data (bytes 20-63) - 44 bytes
    // This includes gyro/accel data and counter that apps can use if supported
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined)

    // Motion Input Timestamp (Microseconds)
    0x09, 0x20,       //   Usage (Vendor Usage 0x20)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0xFF, //   Logical Maximum (655535)
    0x75, 0x20,       //   Report Size (32)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Motion Input Accelerometer XYZ (Gs) and Gyroscope XYZ (Degrees Per Second)
    0x09, 0x21,       //   Usage (Vendor Usage 0x21)
    0x16, 0x00, 0x80, //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, //   Logical Maximum (32767)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x06,       //   Report Count (6)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // Reserved padding
    0x09, 0x22,       //   Usage (Vendor Usage 0x22)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x1D,       //   Report Count (29)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // INPUT REPORT ID 0x02 - Vendor COMMAND data
    0x85, 0x02,       //   Report ID (2)
    0x09, 0x23,       //   Usage (Vendor Usage 0x23)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8 bits)
    0x95, 0x3F,       //   Report Count (63) - 64 bytes minus report ID
    0x81, 0x02,       //   Input (Data,Var,Abs)

    // OUTPUT REPORT ID 0x03 - Vendor COMMAND data
    0x85, 0x03,       //   Report ID (3)
    0x09, 0x24,       //   Usage (Vendor Usage 0x24)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8 bits)
    0x95, 0x2F,       //   Report Count (47) - 48 bytes minus report ID
    0x91, 0x02,       //   Output (Data,Var,Abs)

    0xC0 // End Collection
};

#define SINPUT_REPORT_LEN_INPUT   64
#define TIMER_INTERVAL_MS 8

#pragma pack(push, 1) // Ensure byte alignment
// Input report (Report ID: 1)
typedef struct
{
    uint8_t plug_status;    // Plug Status Format
    uint8_t charge_percent; // 0-100

    union {
        struct {
            uint8_t button_south : 1;
            uint8_t button_east  : 1;
            uint8_t button_west  : 1;
            uint8_t button_north : 1;
            uint8_t dpad_up    : 1;
            uint8_t dpad_down  : 1;
            uint8_t dpad_left  : 1;
            uint8_t dpad_right : 1;
        };
        uint8_t buttons_1;
    };

    union
    {
        struct
        {
            uint8_t button_stick_left : 1;
            uint8_t button_stick_right : 1;
            uint8_t button_l_shoulder : 1;
            uint8_t button_r_shoulder : 1;
            uint8_t button_l_trigger : 1;
            uint8_t button_r_trigger : 1;
            uint8_t button_l_paddle_1 : 1;
            uint8_t button_r_paddle_1 : 1;
        };
        uint8_t buttons_2;
    };

    union
    {
        struct
        {
            uint8_t button_start  : 1;
            uint8_t button_select : 1;
            uint8_t button_guide  : 1;
            uint8_t button_share  : 1;
            uint8_t button_l_paddle_2 : 1;
            uint8_t button_r_paddle_2 : 1;
            uint8_t button_l_touchpad : 1;
            uint8_t button_r_touchpad : 1;
        };
        uint8_t buttons_3;
    };

    union
    {
        struct
        {
            uint8_t button_power   : 1;
            uint8_t button_misc_4  : 1;
            uint8_t button_misc_5  : 1;
            uint8_t button_misc_6  : 1;
            
            // Misc 7 through 10 is unused by
            // SDL currently!
            uint8_t button_misc_7  : 1; 
            uint8_t button_misc_8  : 1;
            uint8_t button_misc_9  : 1;
            uint8_t button_misc_10 : 1;
        };
        uint8_t buttons_4;
    };

    int16_t left_x;             // Left stick X
    int16_t left_y;             // Left stick Y
    int16_t right_x;            // Right stick X
    int16_t right_y;            // Right stick Y
    int16_t trigger_l;          // Left trigger
    int16_t trigger_r;          // Right trigger

    uint32_t imu_timestamp_us;  // IMU Timestamp
    int16_t accel_x;            // Accelerometer X
    int16_t accel_y;            // Accelerometer Y
    int16_t accel_z;            // Accelerometer Z
    int16_t gyro_x;             // Gyroscope X
    int16_t gyro_y;             // Gyroscope Y
    int16_t gyro_z;             // Gyroscope Z

    int16_t touchpad_1_x;       // Touchpad/trackpad
    int16_t touchpad_1_y;
    int16_t touchpad_1_pressure;

    int16_t touchpad_2_x;
    int16_t touchpad_2_y;
    int16_t touchpad_2_pressure;

    uint8_t reserved_bulk[17];  // Reserved for command data
} sinput_input_s;
#pragma pack(pop)

void bluetooth_generate_hid_report(uint8_t *report_data)
{
    static sinput_input_s data = {0};
    static int16_t test = 0;
    data.left_x = test;
    data.left_y = test;

    test+=100;
    if(test > -8000 && test < 8000) test +=16000;
    data.right_x = 0;
    data.right_y = 0;

    memcpy(report_data, &data, 63);
}

void bluetooth_send_report()
{
    if(!hid_cid) return;
    uint8_t report_data[65] = {0};
    report_data[0] = 0xA1;
    report_data[1] = 0x01;
    bluetooth_generate_hid_report(&report_data[2]);
    hid_device_send_interrupt_message(hid_cid, report_data, 65);
}

static void bluetooth_output_report_handler(uint16_t cid,
                                   hid_report_type_t report_type,
                                   uint16_t report_id,
                                   int report_size, uint8_t *report)
{
}

static void bluetooth_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t packet_size)
{
    UNUSED(channel);
    UNUSED(packet_size);
    uint8_t status;
    if (packet_type == HCI_EVENT_PACKET)
    {
        switch (packet[0])
        {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
                return;

            if (!hid_cid)
            {
                //hci_set_bd_addr(bd_addr);

                gap_delete_all_link_keys();
                gap_discoverable_control(1);
            }
            break;

        case HCI_EVENT_USER_CONFIRMATION_REQUEST:
            // ssp: inform about user confirmation request
            printf("SSP User Confirmation Auto accept\n");
            break;
        case HCI_EVENT_HID_META:
            switch (hci_event_hid_meta_get_subevent_code(packet))
            {
            case HID_SUBEVENT_CONNECTION_OPENED:
                status = hid_subevent_connection_opened_get_status(packet);
                if (status)
                {
                    // outgoing connection failed
                    printf("Connection failed, status 0x%x\n", status);
                    gap_discoverable_control(1);
                    hid_cid = 0;
                    return;
                }
                bd_addr_t addr;
                hid_subevent_connection_opened_get_bd_addr(packet, addr);

                printf("HID Connected\n");
                hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                
                break;
            case HID_SUBEVENT_CONNECTION_CLOSED:
                printf("HID Disconnected\n");
                gap_discoverable_control(1);
                hid_cid = 0;
                break;
            case HID_SUBEVENT_CAN_SEND_NOW:
                if (hid_cid)
                {
                    
                }
                break;
            case HID_SUBEVENT_SNIFF_SUBRATING_PARAMS:
            {
                uint16_t max = hid_subevent_sniff_subrating_params_get_host_max_latency(packet);
                uint16_t min = hid_subevent_sniff_subrating_params_get_host_min_timeout(packet);
                printf("Sniff: %d, %d\n", max, min);
            }
            break;

            default:
                break;
            }
            break;
        default:
            break;
        }
    }
}

void main()
{
    stdio_init_all();

    btstack_memory_init();

    bd_addr_t newAddr = {0x7c,
                         0xbb,
                         0x8a,
                         (uint8_t)(get_rand_32() % 0xff),
                         (uint8_t)(get_rand_32() % 0xff),
                         (uint8_t)(get_rand_32() % 0xff)};

    // If the init fails it returns true lol
    if (cyw43_arch_init())
    {
        return;
    }

    //gap_set_bondable_mode(1);

    gap_set_class_of_device(0x2508);
    gap_set_local_name("Test Gamepad");

    uint16_t link_policy = LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE;

    gap_set_default_link_policy_settings(link_policy);
    gap_set_allow_role_switch(true);

    hci_set_chipset(btstack_chipset_cyw43_instance());

    // L2CAP
    l2cap_init();

    sm_init();
    // sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    // sm_set_authentication_requirements(0);

    // SDP Server
    sdp_init();

    hid_sdp_record_t hid_sdp_record = {
        .hid_device_subclass = 0x2508,      // Device Subclass HID
        .hid_country_code = 33,             // Country Code
        .hid_virtual_cable = 1,             // HID Virtual Cable
        .hid_remote_wake = 1,               // HID Remote Wake
        .hid_reconnect_initiate = 1,        // HID Reconnect Initiate
        .hid_normally_connectable = 0,      // HID Normally Connectable
        .hid_boot_device = 0,               // HID Boot Device
        .hid_ssr_host_max_latency = 0xFFFF, // = x * 0.625ms
        .hid_ssr_host_min_timeout = 0xFFFF,
        .hid_supervision_timeout = 3200,                // HID Supervision Timeout
        .hid_descriptor = sinput_hid_report_descriptor, // HID Descriptor
        .hid_descriptor_size = DESCRIPTOR_LEN,                     // HID Descriptor Length
        .device_name = "SInput Test",                   // Device Name
    };

    // Register SDP services

    memset(hid_service_buffer, 0, sizeof(hid_service_buffer));
    hid_create_sdp_record(hid_service_buffer, sdp_create_service_record_handle(), &hid_sdp_record);

    sdp_register_service(hid_service_buffer);

    memset(pnp_service_buffer, 0, sizeof(pnp_service_buffer));

    device_id_create_sdp_record(pnp_service_buffer, sdp_create_service_record_handle(), DEVICE_ID_VENDOR_ID_SOURCE_USB,
                                0x2E8A, 0x10DF, 0x0100);

    sdp_register_service(pnp_service_buffer);

    // HID Device
    hid_device_init(0, DESCRIPTOR_LEN,
                    sinput_hid_report_descriptor);

    hci_event_callback_registration.callback = &bluetooth_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    hid_device_register_packet_handler(&bluetooth_packet_handler);
    hid_device_register_report_data_callback(&bluetooth_output_report_handler);

    hci_power_control(HCI_POWER_ON);
    hci_set_bd_addr(newAddr);

    for(;;)
    {
        sleep_ms(4);
        bluetooth_send_report();
    }
}