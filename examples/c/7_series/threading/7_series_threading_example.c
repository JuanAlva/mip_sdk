////////////////////////////////////////////////////////////////////////////////
/// @file 7_series_threading_example.c
///
/// @defgroup _7_series_threading_example_c 7-Series Threading Example [C]
///
/// @ingroup examples_c
///
/// @brief Example multithreading program for 7-series devices using C
///
/// @details This example shows a basic setup for 7-series devices to
///          demonstrate multithreading for data collection and command updates
///          using C. This is not an exhaustive example of all settings for
///          those devices. If this example does not meet your specific setup
///          needs, please consult the MIP SDK API documentation for the proper
///          commands.
///
/// @section _7_series_threading_example_c_license License
///
/// @copyright Copyright (c) 2025 MicroStrain by HBK
///            Licensed under MIT License
///
/// @{
///

// Include the MicroStrain Serial connection header
#include <microstrain/connections/serial/serial_port.h>

// Include the MicroStrain logging header for custom logging
#include <microstrain/logging.h>

// Include all necessary MIP headers
// Note: The MIP SDK has headers for each module to include all headers associated with the module
// I.E., #include <mip/mip_all.h>
#include <mip/mip_interface.h>
#include <mip/definitions/commands_3dm.h>
#include <mip/definitions/commands_base.h>
#include <mip/definitions/data_filter.h>
#include <mip/definitions/data_sensor.h>
#include <mip/definitions/data_shared.h>

#include <inttypes.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef _MSC_VER
#include <unistd.h>      // isatty, STDIN_FILENO, close
#include <sys/socket.h>  // socket, connect, send
#include <netinet/in.h>  // sockaddr_in, htons
#include <arpa/inet.h>   // inet_pton
#endif // _MSC_VER

// On platforms without MSG_NOSIGNAL (e.g. macOS), silently drop — SIGPIPE blocked via SO_NOSIGPIPE
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif // MSG_NOSIGNAL

#ifdef _MSC_VER
// MSVC doesn't support pthread
// Wrapping basic pthread functionality through threads.h
#include <threads.h>
typedef thrd_t                     pthread_t;
typedef struct pthread_attr_t      pthread_attr_t;
typedef mtx_t                      pthread_mutex_t;
typedef struct pthread_mutexattr_t pthread_mutexattr_t;

int  pthread_mutex_init(pthread_mutex_t* m, const pthread_mutexattr_t* a);
int  pthread_mutex_destroy(pthread_mutex_t* m);
int  pthread_mutex_lock(pthread_mutex_t* m);
int  pthread_mutex_unlock(pthread_mutex_t* m);
int  pthread_create(pthread_t* th, const pthread_attr_t* attr, void* (*func)(void*), void* arg);
int  pthread_join(pthread_t t, void** res);
int  nanosleep(const struct timespec* request, struct timespec* remain);
void sched_yield();
#else
#include <pthread.h>
#endif // _MSC_VER

////////////////////////////////////////////////////////////////////////////////
// NOTE: Setting these globally for example purposes

// TODO: Update to the correct port name and baudrate
/// @brief  Set the port name for the connection (Serial/USB)
#ifdef _WIN32
static const char* PORT_NAME = "COM1";
#else  // Unix
static const char* PORT_NAME = "/dev/ttyUSB0";
#endif // _WIN32

/// @brief  Set the baudrate for the connection (Serial/USB)
/// @note For native serial connections this needs to be 115200 due to the device default settings command
/// Use mip_base_*_comm_speed() to write and save the baudrate on the device
static const uint32_t BAUDRATE = 460800;

// TODO: Update to the desired streaming rate. Setting low for readability purposes
/// @brief Streaming rate in Hz
static const uint16_t SAMPLE_RATE_HZ = 100;

// TODO: Update to change the example run time
/// @brief Example run time
// static const uint32_t RUN_TIME_SECONDS = 20;

// TODO: Enable/disable data collection threading
/// @brief Use this to test the behaviors of threading
#define USE_THREADS true

/// @brief TCP host and port where the Python orchestrator data socket listens
/// @note  Must match SOCKET_HOST / SOCKET_PORT in orchestrator.py
#define SOCKET_HOST "127.0.0.1"
#define SOCKET_PORT 9000
////////////////////////////////////////////////////////////////////////////////

/// @brief Global stop flag — set to 1 by SIGTERM/SIGINT to exit the main loop cleanly
static volatile sig_atomic_t g_stop_requested = 0;

/// @brief TCP socket fd for forwarding MIP data to the Python orchestrator (-1 = not connected)
static int g_socket_fd = -1;

///
/// @} group _7_series_threading_example_c
////////////////////////////////////////////////////////////////////////////////

// Signal handler for clean shutdown (SIGTERM from orchestrator, SIGINT from Ctrl+C)
static void signal_handler(int _signal);

// Data socket — connect to the Python orchestrator and forward MIP packets
static bool connect_data_socket(void);
static void send_socket_msg(uint8_t _type, uint64_t _ns, const float* _values, uint16_t _flags);

// Custom logging handler callback
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args);

// Capture gyro bias
static void capture_gyro_bias(mip_interface* _device);

// Used for basic timestamping (since epoch in milliseconds)
// TODO: Update this to whatever timestamping method is desired
static mip_timestamp get_current_timestamp();

// Device callbacks used for reading and writing packets
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length);
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
);

// Common device initialization procedure
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate);

// Message format configuration
static void configure_sensor_message_format(mip_interface* _device);
static void configure_filter_message_format(mip_interface* _device);

// Packet callback handler
static void packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp);

#if USE_THREADS
// Basic structure for thread data
typedef struct thread_data
{
    mip_interface* device;
    volatile bool  running;
} thread_data_t;

// Threaded functions
static bool  update_device(mip_interface* _device, mip_timeout _wait_time, bool _from_cmd);
static void* data_collection_thread(void* _thread_data);
#endif // USE_THREADS

// Utility functions the handle application closing and printing error messages
static void terminate(serial_port* _device_port, const char* _message, const bool _successful);
static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...);

int main(const int argc, const char* argv[])
{
    // Unused parameters
    (void)argc;
    (void)argv;

    // Mark printf operations as unbuffered to flush with every operation
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // Register signal handlers for clean shutdown from the Python orchestrator or Ctrl+C
    signal(SIGTERM, signal_handler);
    signal(SIGINT,  signal_handler);

// Note: This is a compile-time way of checking that the proper logging level is enabled
// Note: The max available logging level may differ in pre-packaged installations of the MIP SDK
#ifndef MICROSTRAIN_LOGGING_ENABLED_INFO
#error This example requires a logging level of at least MICROSTRAIN_LOGGING_LEVEL_INFO_ to work properly
#endif // !MICROSTRAIN_LOGGING_ENABLED_INFO

#if USE_THREADS
    // Create a mutex for the logging callbacks when multi-threading
    fprintf(stdout, "Initializing the threading mutex.\n");
    pthread_mutex_t lock;
    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr, "Failed to initialize the threading mutex!\n");

        fprintf(stdout, "Press 'Enter' to exit the program.\n");

        // Make sure the console remains open
        const int confirm_exit = getc(stdin);
        (void)confirm_exit; // Unused

        return 1;
    }
#endif // USE_THREADS

    // Initialize the custom logger to print messages/errors as they occur
    // Note: The logging level parameter doesn't need to match the max logging level.
    // If the parameter is higher than the max level, higher-level logging functions will be ignored
#if USE_THREADS
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, (void*)&lock);
#else
    MICROSTRAIN_LOG_INIT(&log_callback, MICROSTRAIN_LOG_LEVEL_INFO, NULL);
#endif // USE_THREADS

    // Initialize the connection
    MICROSTRAIN_LOG_INFO("Initializing the connection.\n");
    serial_port device_port;
    serial_port_init(&device_port);

    MICROSTRAIN_LOG_INFO("Connecting to the device on port %s with %d baudrate.\n", PORT_NAME, BAUDRATE);

    // Open the connection to the device
    if (!serial_port_open(&device_port, PORT_NAME, BAUDRATE))
    {
        terminate(&device_port, "Could not open the connection!\n", false);
    }

    mip_interface device;
    initialize_device(&device, &device_port, BAUDRATE);

    // Capture gyro bias
    capture_gyro_bias(&device);

    // Configure the message format for sensor data
    configure_sensor_message_format(&device);

    // Configure the message format for filter data (quaternion)
    configure_filter_message_format(&device);

    // Sensor and filter data packet callbacks
    MICROSTRAIN_LOG_INFO("Registering sensor and filter data packet callbacks.\n");

    mip_dispatch_handler sensor_packet_handler;
    mip_dispatch_handler filter_packet_handler;

    // Register the callback for sensor packets
    mip_interface_register_packet_callback(
        &device,
        &sensor_packet_handler,
        MIP_SENSOR_DATA_DESC_SET, // Data descriptor set
        false,                    // Process after field callback
        &packet_callback,         // Callback
        NULL                      // User data
    );

    // Register the callback for filter packets
    mip_interface_register_packet_callback(
        &device,
        &filter_packet_handler,
        MIP_FILTER_DATA_DESC_SET, // Data descriptor set
        false,                    // Process after field callback
        &packet_callback,         // Callback
        NULL                      // User data
    );

    // Connect to the data forwarding socket on the Python orchestrator (non-fatal)
#ifndef _MSC_VER
    MICROSTRAIN_LOG_INFO("Connecting to data socket %s:%d...\n", SOCKET_HOST, SOCKET_PORT);
    if (!connect_data_socket())
    {
        MICROSTRAIN_LOG_WARN("Data socket unavailable — data will only be logged locally.\n");
    }
#endif // _MSC_VER

#if USE_THREADS
    MICROSTRAIN_LOG_INFO("Initializing the device update function for threading.\n");
    // Note: This allows the update function to be split into command and data updates across multiple threads
    mip_interface_set_update_function(&device, &update_device);

    thread_data_t data = {.device = &device, .running = true};

    MICROSTRAIN_LOG_INFO("Creating the data collection thread.\n");
    pthread_t data_thread;
    pthread_create(&data_thread, NULL, data_collection_thread, (void*)&data);
#endif // USE_THREADS

    // Resume the device
    // Note: Since the device was idled for configuration, it needs to be resumed to output the data streams
    MICROSTRAIN_LOG_INFO("Resuming the device.\n");
    const mip_cmd_result cmd_result = mip_base_resume(&device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(&device, cmd_result, "Could not resume the device!\n");
    }

    MICROSTRAIN_LOG_INFO("The device is configured... waiting for data.\n");
    // MICROSTRAIN_LOG_INFO("This example will now output data for %ds.\n", RUN_TIME_SECONDS);

    // Get the start time of the device update loop to handle exiting the application
    // const mip_timestamp loop_start_time = get_current_timestamp();

    // Main loop — exits on SIGTERM/SIGINT or after RUN_TIME_SECONDS
    while (!g_stop_requested){ //&&
    //        get_current_timestamp() - loop_start_time <= RUN_TIME_SECONDS * 1000)
    // {
#if USE_THREADS
        // Data collection thread drives all device reads; main thread just idles
        const struct timespec ts_idle = { .tv_sec = 0, .tv_nsec = 100 * 1000000 }; // 100 ms
        nanosleep(&ts_idle, NULL);
#else
        // Without a data collection thread, drive device reads from the main loop
        mip_interface_update(&device, 10, false);
#endif // USE_THREADS
    }

    if (g_stop_requested)
    {
        MICROSTRAIN_LOG_INFO("Stop requested by signal — shutting down.\n");
    }

#if USE_THREADS
    // Signal the data collection thread to stop
    data.running = false;

    // Join the thread back before exiting the program
    MICROSTRAIN_LOG_INFO("Waiting for the thread to join.\n");
    void* thread_return_code = NULL; // Return code from the thread function (Unused)
    if (pthread_join(data_thread, &thread_return_code) != 0)
    {
        pthread_mutex_destroy(&lock);
        free(thread_return_code);
        terminate(&device_port, "Failed to join the thread!\n", false);
    }

    pthread_mutex_destroy(&lock);
    free(thread_return_code);
#endif // USE_THREADS

#ifndef _MSC_VER
    if (g_socket_fd >= 0)
    {
        close(g_socket_fd);
        g_socket_fd = -1;
    }
#endif // _MSC_VER

    terminate(&device_port, "Example Completed Successfully.\n", true);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup _7_series_threading_example_c
/// @{
///

////////////////////////////////////////////////////////////////////////////////
/// @brief Signal handler for clean shutdown
///
/// @details Sets the global stop flag when SIGTERM or SIGINT is received.
///          The main loop and the ping stress-test loop both check this flag
///          so the program exits at the next safe point without corrupting data.
///
/// @param _signal Signal number received (SIGTERM or SIGINT)
///
static void signal_handler(int _signal)
{
    (void)_signal;
    g_stop_requested = 1;
}

#ifndef _MSC_VER
////////////////////////////////////////////////////////////////////////////////
/// @brief Opens a TCP connection to the Python orchestrator data socket
///
/// @details Connects to SOCKET_HOST:SOCKET_PORT. Non-fatal — if the server is
///          not running, g_socket_fd stays -1 and send_socket_msg is a no-op.
///
/// @return True if connected successfully, false otherwise
///
static bool connect_data_socket(void)
{
    g_socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (g_socket_fd < 0)
    {
        MICROSTRAIN_LOG_ERROR("Failed to create socket.\n");
        return false;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(SOCKET_PORT);

    if (inet_pton(AF_INET, SOCKET_HOST, &addr.sin_addr) <= 0)
    {
        MICROSTRAIN_LOG_ERROR("Invalid socket address: %s\n", SOCKET_HOST);
        close(g_socket_fd);
        g_socket_fd = -1;
        return false;
    }

    if (connect(g_socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        MICROSTRAIN_LOG_ERROR("Failed to connect to socket %s:%d.\n", SOCKET_HOST, SOCKET_PORT);
        close(g_socket_fd);
        g_socket_fd = -1;
        return false;
    }

    MICROSTRAIN_LOG_INFO("Data socket connected to %s:%d.\n", SOCKET_HOST, SOCKET_PORT);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Serialises and sends one MIP data message over the TCP socket
///
/// @details Wire format — little-endian, 27 bytes total:
///   Byte  0    : msg_type  (0x01 = scaled accel | 0x02 = attitude quaternion)
///   Bytes 1–8  : device_ns (uint64_t, nanoseconds from device reference time)
///   Bytes 9–24 : values    (4 × float32)
///                  accel : [x, y, z, 0.0]  (units: g)
///                  quat  : [w, x, y, z]
///   Bytes 25–26: flags     (uint16_t, valid_flags for quat; 0 for accel)
///
/// If the socket is broken, it is closed and g_socket_fd is set to -1.
///
/// @param _type    Message type (0x01 or 0x02)
/// @param _ns      Device reference timestamp [nanoseconds]
/// @param _values  Four float32 values
/// @param _flags   Valid-flags word
///
static void send_socket_msg(uint8_t _type, uint64_t _ns, const float* _values, uint16_t _flags)
{
    if (g_socket_fd < 0) return;

    uint8_t buf[27];
    size_t  off = 0;

    buf[off++] = _type;
    memcpy(buf + off, &_ns,    8);  off += 8;
    memcpy(buf + off, _values, 16); off += 16;
    memcpy(buf + off, &_flags, 2);

    if (send(g_socket_fd, buf, sizeof(buf), MSG_NOSIGNAL) < 0)
    {
        // Socket broken — disable forwarding until reconnect
        close(g_socket_fd);
        g_socket_fd = -1;
    }
}
#endif // _MSC_VER

////////////////////////////////////////////////////////////////////////////////
/// @brief Custom logging callback for MIP SDK message formatting and output
///
/// @details Processes and formats log messages from the MIP SDK based on
///          severity level. Routes messages to appropriate output streams -
///          errors and fatal messages go to stderr while other levels go to
///          stdout. Each message is prefixed with its severity level name.
///
/// @param _user Pointer to the threading mutex (if threading is enabled)
/// @param _level Log message severity level from microstrain_log_level enum
/// @param _format Printf-style format string for the message
/// @param _args Variable argument list containing message parameters
///
static void log_callback(void* _user, const microstrain_log_level _level, const char* _format, va_list _args)
{
#if USE_THREADS
    pthread_mutex_t* lock = (pthread_mutex_t*)_user;
    assert(lock);

    // Lock the mutex since the callbacks can happen across threads
    pthread_mutex_lock(lock);
#else
    // Unused parameter
    (void)_user;
#endif // USE_THREADS

    switch (_level)
    {
        case MICROSTRAIN_LOG_LEVEL_FATAL:
        case MICROSTRAIN_LOG_LEVEL_ERROR:
        {
            fprintf(stderr, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stderr, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_WARN:
        case MICROSTRAIN_LOG_LEVEL_INFO:
        case MICROSTRAIN_LOG_LEVEL_DEBUG:
        case MICROSTRAIN_LOG_LEVEL_TRACE:
        {
            fprintf(stdout, "%s: ", microstrain_logging_level_name(_level));
            vfprintf(stdout, _format, _args);
            break;
        }
        case MICROSTRAIN_LOG_LEVEL_OFF:
        default:
        {
            break;
        }
    }

#if USE_THREADS
    // Release the logging callback for other threads
    pthread_mutex_unlock(lock);
#endif // USE_THREADS
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Captures and configures device gyro bias
///
/// @param _device Pointer to the initialized MIP device interface
///
///
static void capture_gyro_bias(mip_interface* _device)
{
    // Get the command queue so we can increase the reply timeout during the capture duration,
    // then reset it afterward
    mip_cmd_queue*    cmd_queue        = mip_interface_cmd_queue(_device);
    const mip_timeout previous_timeout = mip_cmd_queue_base_reply_timeout(cmd_queue);
    MICROSTRAIN_LOG_INFO("Initial command reply timeout is %dms.\n", previous_timeout);

    // Note: The default is 15 s (15,000 ms)
    const uint16_t capture_duration            = 15000;
    const uint16_t increased_cmd_reply_timeout = capture_duration + 1000;

    MICROSTRAIN_LOG_INFO("Increasing command reply timeout to %dms for capture gyro bias.\n", increased_cmd_reply_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, increased_cmd_reply_timeout);

    mip_vector3f gyro_bias = {
        0.0f, // X
        0.0f, // Y
        0.0f  // Z
    };

    // Note: When capturing gyro bias, the device needs to remain still on a flat surface
    MICROSTRAIN_LOG_WARN("About to capture gyro bias for %.2g seconds!\n", (float)capture_duration / 1000.0f);
    MICROSTRAIN_LOG_WARN("Please do not move the device during this time!\n");
    // MICROSTRAIN_LOG_WARN("Press 'Enter' when ready...");

    // // Wait for anything to be entered
    // const int confirm_capture = getc(stdin);
    // (void)confirm_capture; // Unused

    MICROSTRAIN_LOG_WARN("Capturing gyro bias...\n");
    const mip_cmd_result cmd_result = mip_3dm_capture_gyro_bias(
        _device,
        capture_duration, // Capture duration (ms)
        gyro_bias         // Gyro bias out (result of the capture)
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Failed to capture gyro bias!\n");
    }

    MICROSTRAIN_LOG_INFO("Capture gyro bias completed with result: [%f, %f, %f]\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    MICROSTRAIN_LOG_INFO("Reverting command reply timeout to %dms.\n", previous_timeout);
    mip_cmd_queue_set_base_reply_timeout(cmd_queue, previous_timeout);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Gets the current system timestamp in milliseconds
///
/// @details Provides basic timestamping using system time:
///          - Returns milliseconds since Unix epoch
///          - Uses timespec_get() with UTC time base
///          - Returns 0 if time cannot be obtained
///
/// @note Update this function to use a different time source if needed for
///       your specific application requirements
///
/// @return Current system time in milliseconds since epoch
///
static mip_timestamp get_current_timestamp()
{
    struct timespec ts;

    // Get system UTC time since epoch
    if (timespec_get(&ts, TIME_UTC) != TIME_UTC)
    {
        return 0;
    }

    // Get the time in milliseconds
    return (mip_timestamp)ts.tv_sec * 1000 + (mip_timestamp)ts.tv_nsec / 1000000;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles sending packets to the device
///
/// @details Implements the MIP device interface send callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Writes data buffer to serial port
///
/// @param _device MIP device interface containing the connection
/// @param _data Buffer containing packet data to send
/// @param _length Number of bytes to send
///
/// @return True if send was successful, false otherwise
///
static bool mip_interface_user_send_to_device(mip_interface* _device, const uint8_t* _data, size_t _length)
{
    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the bytes written to the device
    size_t bytes_written;

    // Send the packet to the device
    return serial_port_write(device_port, _data, _length, &bytes_written);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles receiving packets from the device
///
/// @details Implements the MIP device interface receive callback:
///          - Extracts serial port from device user pointer
///          - Validates connection state
///          - Reads available data into provided buffer
///          - Timestamps the received data
///
/// @param _device MIP device interface containing the connection
/// @param _buffer Buffer to store received data
/// @param _max_length Maximum number of bytes to read
/// @param _wait_time How long to wait for data in milliseconds
/// @param _from_cmd Whether this read is from a command response (unused)
/// @param _length_out Number of bytes actually read
/// @param _timestamp_out Timestamp when data was received
///
/// @return True if receive was successful, false otherwise
///
static bool mip_interface_user_recv_from_device(
    mip_interface* _device, uint8_t* _buffer, size_t _max_length, mip_timeout _wait_time, bool _from_cmd,
    size_t* _length_out, mip_timestamp* _timestamp_out
)
{
    // Unused parameter
    (void)_from_cmd;

    // Extract the serial port pointer that was used in the callback initialization
    serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

    if (device_port == NULL)
    {
        MICROSTRAIN_LOG_ERROR("serial_port pointer not set in mip_interface_init().\n");
        return false;
    }

    // Get the time that the packet was received (system epoch UTC time in milliseconds)
    *_timestamp_out = get_current_timestamp();

    // Read the packet from the device
    return serial_port_read(device_port, _buffer, _max_length, (int)_wait_time, _length_out);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes and configures a MIP device interface
///
/// @details Performs a complete device initialization sequence:
///          1. Sets up a MIP device interface with specified timeouts and
///             callbacks
///          2. Verifies device communication with a ping command
///          3. Sets the device to idle mode to ensure reliable configuration
///          4. Queries and displays detailed device information
///          5. Loads default device settings for a known state
///
/// @param _device Pointer to a MIP device interface to initialize
/// @param _device_port Pointer to an initialized serial port for device
///                     communication
/// @param _baudrate Serial communication baudrate for the device
///
static void initialize_device(mip_interface* _device, serial_port* _device_port, const uint32_t _baudrate)
{
    MICROSTRAIN_LOG_INFO("Initializing the device interface.\n");
    mip_interface_init(
        _device,
        mip_timeout_from_baudrate(_baudrate), // Set the base timeout for commands (milliseconds)
        2000,                                 // Set the base timeout for command replies (milliseconds)
        &mip_interface_user_send_to_device,   // User-defined send packet callback
        &mip_interface_user_recv_from_device, // User-defined receive packet callback
        &mip_interface_default_update,        // Default update callback
        (void*)_device_port                   // Cast the device port for use in the callbacks
    );

    // Ping the device
    // Note: This is a good first step to make sure the device is present
    MICROSTRAIN_LOG_INFO("Pinging the device.\n");
    mip_cmd_result cmd_result = mip_base_ping(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not ping the device!\n");
    }

    // Set the device to Idle
    // Note: This is good to do during setup as high data traffic can cause commands to fail
    MICROSTRAIN_LOG_INFO("Setting the device to idle.\n");
    cmd_result = mip_base_set_idle(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not set the device to idle!\n");
    }

    // Print device info to make sure the correct device is being used
    MICROSTRAIN_LOG_INFO("Getting the device information.\n");
    mip_base_device_info device_info;
    cmd_result = mip_base_get_device_info(_device, &device_info);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the device information!\n");
    }

    // Extract the major minor and patch values
    const uint16_t major = device_info.firmware_version / 1000;
    const uint16_t minor = device_info.firmware_version / 100 % 10;
    const uint16_t patch = device_info.firmware_version % 100;

    // Firmware version format is x.x.xx
    char firmwareVersion[16] = {0};
    snprintf(firmwareVersion, sizeof(firmwareVersion) / sizeof(firmwareVersion[0]), "%d.%d.%02d", major, minor, patch);

    MICROSTRAIN_LOG_INFO("-------- Device Information --------\n");
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Name", device_info.model_name);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Model Number", device_info.model_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Serial Number", device_info.serial_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Lot Number", device_info.lot_number);
    MICROSTRAIN_LOG_INFO("%-16s | %.16s\n", "Options", device_info.device_options);
    MICROSTRAIN_LOG_INFO("%-16s | %16s\n", "Firmware Version", firmwareVersion);
    MICROSTRAIN_LOG_INFO("------------------------------------\n");

    // Load the default settings on the device
    // Note: This guarantees the device is in a known state
    MICROSTRAIN_LOG_INFO("Loading device settings.\n");
    cmd_result = mip_3dm_load_device_settings(_device);

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        // Note: Default settings will reset the baudrate to 115200 and may cause connection issues
        if (cmd_result == MIP_STATUS_TIMEDOUT && BAUDRATE != 460800)
        {
            MICROSTRAIN_LOG_WARN("On a native serial connections the baudrate needs to be 115200 for this example to run.\n");
        }

        exit_from_command(_device, cmd_result, "Could not load device default settings!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for sensor data streaming
///
/// @details Sets up sensor data output by:
///          1. Querying device base rate
///          2. Validating desired sample rate against base rate
///          3. Calculating proper decimation
///          4. Configuring message format with:
///             - Scaled accelerometer
///
/// @param _device Pointer to the initialized MIP device interface
///
static void configure_sensor_message_format(mip_interface* _device)
{
    // Note: Querying the device base rate is only one way to calculate the descriptor decimation
    // We could have also set it directly with information from the datasheet

    MICROSTRAIN_LOG_INFO("Getting the base rate for sensor data.\n");
    uint16_t       sensor_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_SENSOR_DATA_DESC_SET, // Data descriptor set
        &sensor_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the base rate for sensor data!\n");
    }

    // Supported sample rates can be any value from 1 up to the base rate
    // Note: Decimation can be anything from 1 to 65,565 (uint16_t::max)
    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > sensor_base_rate)
    {
        exit_from_command(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            sensor_base_rate
        );
    }

    // Calculate the decimation (stream rate) for the device based on its base rate
    const uint16_t sensor_decimation = sensor_base_rate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating sensor base rate %d by %d to stream data at %dHz.\n",
        sensor_base_rate,
        sensor_decimation,
        SAMPLE_RATE_HZ
    );

    // Descriptor rate is a pair of data descriptor set and decimation
    const mip_descriptor_rate sensor_descriptors[2] = {
        {MIP_DATA_DESC_SHARED_REFERENCE_TIME, sensor_decimation}, // Device internal timestamp (ns)
        {MIP_DATA_DESC_SENSOR_ACCEL_SCALED,   sensor_decimation}
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for sensor data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_SENSOR_DATA_DESC_SET,                                   // Data descriptor set
        sizeof(sensor_descriptors) / sizeof(sensor_descriptors[0]), // Number of descriptors to include
        sensor_descriptors                                          // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure message format for sensor data!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Configures message format for filter data streaming
///
/// @details Sets up filter data output with:
///          - Attitude quaternion (q_w, q_x, q_y, q_z)
///
/// @param _device Pointer to the initialized MIP device interface
///
static void configure_filter_message_format(mip_interface* _device)
{
    MICROSTRAIN_LOG_INFO("Getting the base rate for filter data.\n");
    uint16_t       filter_base_rate;
    mip_cmd_result cmd_result = mip_3dm_get_base_rate(
        _device,
        MIP_FILTER_DATA_DESC_SET, // Data descriptor set
        &filter_base_rate         // Base rate out
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not get the base rate for filter data!\n");
    }

    if (SAMPLE_RATE_HZ == 0 || SAMPLE_RATE_HZ > filter_base_rate)
    {
        exit_from_command(
            _device,
            MIP_NACK_INVALID_PARAM,
            "Invalid sample rate of %dHz! Supported rates are [1, %d].\n",
            SAMPLE_RATE_HZ,
            filter_base_rate
        );
    }

    const uint16_t filter_decimation = filter_base_rate / SAMPLE_RATE_HZ;
    MICROSTRAIN_LOG_INFO(
        "Decimating filter base rate %d by %d to stream data at %dHz.\n",
        filter_base_rate,
        filter_decimation,
        SAMPLE_RATE_HZ
    );

    const mip_descriptor_rate filter_descriptors[2] = {
        {MIP_DATA_DESC_SHARED_REFERENCE_TIME,  filter_decimation}, // Device internal timestamp (ns)
        {MIP_DATA_DESC_FILTER_ATT_QUATERNION,  filter_decimation}
    };

    MICROSTRAIN_LOG_INFO("Configuring message format for filter data.\n");
    cmd_result = mip_3dm_write_message_format(
        _device,
        MIP_FILTER_DATA_DESC_SET,                                       // Data descriptor set
        sizeof(filter_descriptors) / sizeof(filter_descriptors[0]),     // Number of descriptors
        filter_descriptors                                              // Descriptor array
    );

    if (!mip_cmd_result_is_ack(cmd_result))
    {
        exit_from_command(_device, cmd_result, "Could not configure message format for filter data!\n");
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Callback function that processes received MIP packets
///
/// @details This function is called whenever a MIP packet is received from the
///          device.
///          It processes the packet by:
///          1. Extracting all fields from the packet
///          2. Building a formatted string of field descriptors
///          3. Logging packet information including timestamp, descriptor set,
///             and field descriptors
///
/// @param _user Pointer to user data (unused in this implementation)
/// @param _packet_view Pointer to the received MIP packet
/// @param _timestamp Timestamp when the packet was received
///
static void packet_callback(void* _user, const mip_packet_view* _packet_view, mip_timestamp _timestamp)
{
    // Unused parameters
    (void)_user;
    (void)_timestamp;

    // Field object for iterating the packet
    mip_field_view field_view;
    mip_field_init_empty(&field_view);

    // First pass: extract the device reference timestamp embedded in the packet
    // This timestamp is set by the device at sample time (1 ns resolution, monotonic)
    // and is far more precise than the PC receive timestamp (_timestamp)
    uint64_t device_ns = 0;

    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        if (mip_field_field_descriptor(&field_view) == MIP_DATA_DESC_SHARED_REFERENCE_TIME)
        {
            mip_shared_reference_timestamp_data ref_ts;

            if (extract_mip_shared_reference_timestamp_data_from_field(&field_view, (void*)&ref_ts))
            {
                device_ns = ref_ts.nanoseconds;
            }

            break;
        }
    }

    // Second pass: extract and print data fields using the device timestamp
    mip_field_init_empty(&field_view);

    while (mip_field_next_in_packet(&field_view, _packet_view))
    {
        if (mip_field_field_descriptor(&field_view) == MIP_DATA_DESC_SENSOR_ACCEL_SCALED)
        {
            mip_sensor_scaled_accel_data accel_data;

            if (extract_mip_sensor_scaled_accel_data_from_field(&field_view, (void*)&accel_data))
            {
                MICROSTRAIN_LOG_INFO(
                    "Scaled Accel at %" PRIu64 " ns (0x%02X, 0x%02X): [%9.6f, %9.6f, %9.6f] g\n",
                    device_ns,
                    MIP_SENSOR_DATA_DESC_SET,
                    MIP_DATA_DESC_SENSOR_ACCEL_SCALED,
                    accel_data.scaled_accel[0],
                    accel_data.scaled_accel[1],
                    accel_data.scaled_accel[2]
                );

#ifndef _MSC_VER
                const float accel_values[4] = {
                    accel_data.scaled_accel[0],
                    accel_data.scaled_accel[1],
                    accel_data.scaled_accel[2],
                    0.0f
                };
                send_socket_msg(0x01, device_ns, accel_values, 0);
#endif // _MSC_VER
            }
        }
        else if (mip_field_field_descriptor(&field_view) == MIP_DATA_DESC_FILTER_ATT_QUATERNION)
        {
            mip_filter_attitude_quaternion_data quat_data;

            if (extract_mip_filter_attitude_quaternion_data_from_field(&field_view, (void*)&quat_data))
            {
                MICROSTRAIN_LOG_INFO(
                    "Attitude Quat  at %" PRIu64 " ns (0x%02X, 0x%02X): [w=%9.6f, x=%9.6f, y=%9.6f, z=%9.6f] flags=0x%04X\n",
                    device_ns,
                    MIP_FILTER_DATA_DESC_SET,
                    MIP_DATA_DESC_FILTER_ATT_QUATERNION,
                    quat_data.q[0],
                    quat_data.q[1],
                    quat_data.q[2],
                    quat_data.q[3],
                    quat_data.valid_flags
                );

#ifndef _MSC_VER
                const float quat_values[4] = {
                    quat_data.q[0],
                    quat_data.q[1],
                    quat_data.q[2],
                    quat_data.q[3]
                };
                send_socket_msg(0x02, device_ns, quat_values, quat_data.valid_flags);
#endif // _MSC_VER
            }
        }
    }
}

#if USE_THREADS
////////////////////////////////////////////////////////////////////////////////
/// @brief Updates the device state based on command or data collection context
///
/// @details Handles device updates differently depending on whether called from
///          command handling or data collection:
///          - For data collection (_from_cmd = false): Performs normal device
///            updates
///          - For commands (_from_cmd = true): Sleeps briefly to save power
///            while avoiding command timeouts
///
/// @param _device Pointer to the MIP device interface
/// @param _wait_time Time to wait for updates (typically used only from
///                   commands)
/// @param _from_cmd True if called from command handling, false for data
///                  collection
///
/// @returns true if the update was successful, false on error.
///          Always returns true when called from commands to avoid race
///          conditions.
///
static bool update_device(mip_interface* _device, mip_timeout _wait_time, bool _from_cmd)
{
    // Do normal updates only if not called from a command handler
    // Note: This is the separation between the main/other thread and the data collection thread
    if (!_from_cmd)
    {
        return mip_interface_default_update(_device, _wait_time, _from_cmd);
    }

    // Create a 5-millisecond timeout
    const struct timespec ts = {
        .tv_sec  = 0,          // 0 Seconds
        .tv_nsec = 5 * 1000000 // 5 Milliseconds
    };

    // Sleep for a bit to save power
    // Note: Waiting too long in here will cause commands to timeout
    nanosleep(&ts, NULL);

    // Note: This needs to return true to avoid terminating the data collection thread
    // Note: Returning false may cause a race condition (see comments in mip_interface_wait_for_reply)
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles continuous data collection from the device in a separate
///        thread
///
/// @details Main function for the data collection thread that:
///          1. Continuously updates device state for receiving data
///          2. Clears command queue if the connection closes to avoid
///             deadlocks
///          3. Yields thread time when possible
///          4. Runs until the running state becomes false
///
/// @param _thread_data Pointer to a thread data structure containing:
///                     - device: MIP device interface
///                     - running: boolean controlling thread execution
///
/// @returns NULL (return value unused)
///
static void* data_collection_thread(void* _thread_data)
{
    MICROSTRAIN_LOG_INFO("Data collection thread created!\n");

    const thread_data_t* thread_data = (thread_data_t*)_thread_data;

    while (thread_data->running)
    {
        // Update the device for data collection
        // Note: The recommended default wait time is 10 ms, but could be 0 for non-blocking read operations
        const bool updated = mip_interface_update(
            thread_data->device,
            10,   // Time to wait
            false // From command
        );

        // Clean up and exit the thread on failed device updates
        if (!updated)
        {
            // Avoid deadlocks if the connection is closed
            mip_cmd_queue* cmd_queue = mip_interface_cmd_queue(thread_data->device);
            assert(cmd_queue);
            mip_cmd_queue_clear(cmd_queue);

            break;
        }

        sched_yield();
    }

    // Return value unused
    return NULL;
}
#endif // USE_THREADS

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and cleanup
///
/// @details Handles graceful shutdown when errors occur:
///          - Outputs provided error message
///          - Closes device connection if open
///          - Exits with appropriate status code
///
/// @param _device_port Serial port connection to close
/// @param _message Error message to display
/// @param _successful Whether termination is due to success or failure
///
static void terminate(serial_port* _device_port, const char* _message, const bool _successful)
{
    if (_message != NULL && strlen(_message) != 0)
    {
        if (_successful)
        {
            MICROSTRAIN_LOG_INFO("%s", _message);
        }
        else
        {
            MICROSTRAIN_LOG_ERROR("%s", _message);
        }
    }

    if (_device_port == NULL)
    {
        // Initialize the device interface with a serial port connection
        MICROSTRAIN_LOG_ERROR("Connection not set for the device interface. Cannot close the connection.\n");
    }
    else
    {
        if (serial_port_is_open(_device_port))
        {
            MICROSTRAIN_LOG_INFO("Closing the connection.\n");

            if (!serial_port_close(_device_port))
            {
                MICROSTRAIN_LOG_ERROR("Failed to close the connection!\n");
            }
        }
    }

#ifndef _MSC_VER
    // Only wait for user input when running in an interactive terminal.
    // When launched from the Python orchestrator, stdin is not a tty — skip the prompt.
    if (isatty(STDIN_FILENO))
#endif // _MSC_VER
    {
        MICROSTRAIN_LOG_INFO("Press 'Enter' to exit the program.\n");
        const int confirm_exit = getc(stdin);
        (void)confirm_exit; // Unused
    }

    if (!_successful)
    {
        exit(1);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Handles graceful program termination and command failure cleanup
///
/// @details Handles command failure scenarios:
///          - Formats and displays an error message with command result
///          - Closes device connection
///          - Exits with failure status
///
/// @param _device MIP device interface for the command that failed
/// @param _cmd_result Result code from a failed command
/// @param _format Printf-style format string for error message
/// @param ... Variable arguments for format string
///
static void exit_from_command(const mip_interface* _device, const mip_cmd_result _cmd_result, const char* _format, ...)
{
    if (_format != NULL && strlen(_format) != 0)
    {
        va_list args;
        va_start(args, _format);
        MICROSTRAIN_LOG_ERROR_V(_format, args);
        va_end(args);
    }

    MICROSTRAIN_LOG_ERROR("Command Result: (%d) %s.\n", _cmd_result, mip_cmd_result_to_string(_cmd_result));

    if (_device == NULL)
    {
        terminate(NULL, "", false);
    }
    else
    {
        // Get the connection pointer that was set during device initialization
        serial_port* device_port = (serial_port*)mip_interface_user_pointer(_device);

        terminate(device_port, "", false);
    }
}

///
/// @} group _7_series_threading_example_c
////////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
// threads.h wrappers for unsupported pthread functionality used in this example

// threads.h wrapper for pthread pthread_mutex_init
int pthread_mutex_init(pthread_mutex_t* m, const pthread_mutexattr_t* a)
{
    // Unused parameter
    (void)a;

    // Simple non-recursive mutex
    return mtx_init(m, mtx_plain);
}

// threads.h wrapper for pthread pthread_mutex_destroy
int pthread_mutex_destroy(pthread_mutex_t* m)
{
    mtx_destroy(m);
    return 0;
}

// threads.h wrapper for pthread pthread_mutex_lock
int pthread_mutex_lock(pthread_mutex_t* m)
{
    return mtx_lock(m);
}

// threads.h wrapper for pthread pthread_mutex_unlock
int pthread_mutex_unlock(pthread_mutex_t* m)
{
    return mtx_unlock(m);
}

// threads.h wrapper for pthread pthread_create
int pthread_create(pthread_t* th, const pthread_attr_t* attr, void* (*func)(void*), void* arg)
{
    // Unused parameter
    (void)attr;

    return thrd_create(th, (thrd_start_t)func, arg);
}

// threads.h wrapper for pthread pthread_join
int pthread_join(pthread_t t, void** res)
{
    *res = malloc(sizeof(int));
    return thrd_join(t, *res);
}

// sleep wrapper for pthread nanosleep
int nanosleep(const struct timespec* request, struct timespec* remain)
{
    // Sleep for some duration
    return thrd_sleep(request, remain);
}

// threads.h wrapper for pthread sched_yield
void sched_yield()
{
    thrd_yield();
}
#endif // _MSC_VER
