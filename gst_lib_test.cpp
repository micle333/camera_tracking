#include <gst/gst.h>
#include <gst/video/video.h>
#include <iostream>

int main(int argc, char *argv[]) {
    // Инициализация GStreamer
    gst_init(&argc, &argv);

    // Включаем отладочный вывод GStreamer
    gst_debug_set_active(TRUE);
    gst_debug_set_default_threshold(GST_LEVEL_INFO);

    // Создание GStreamer пайплайна для захвата видео с камеры и вывода на экран
    //193 
    std::string pipeline_str = "v4l2src device=/dev/video0 ! videoconvert ! xvimagesink";
    //193 std::string pipeline_str = "v4l2src device=/dev/video0 ! videoconvert ! glimagesink";
    


    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline || error) {
        std::cerr << "Ошибка запуска пайплайна: " << (error ? error->message : "Unknown error") << std::endl;
        return -1;
    }

    // Запуск пайплайна
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Ожидаем окончания работы пайплайна
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg;
    do {
        msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    } while (!msg);

    // Очистка
    gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    return 0;
}
