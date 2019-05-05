#include <map>
#include <chrono>
#include <thread>
#include <vector>
#include <imgui.h>
#include "example.hpp"
#include "imgui_impl_glfw.h"
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

static const int flags = ImGuiWindowFlags_NoCollapse
       | ImGuiWindowFlags_NoScrollbar
       | ImGuiWindowFlags_NoSavedSettings
       | ImGuiWindowFlags_NoTitleBar
       | ImGuiWindowFlags_NoResize
       | ImGuiWindowFlags_NoMove;

struct view {
    std::string     serial_num;
    rs2::frameset   frame;
};

void sync (rs2::device *, int);
void init (window *);
void button();
void play(window , rs2::device, rs2::pipeline *);
void pause (window ,rs2::device , rs2::pipeline &,int);
void draw_seek_bar(rs2::playback& , int* , float2& , float );
std::string pretty_time(std::chrono::nanoseconds duration);

int main(int argc, char ** argv) try
{
    window app(1280, 960, "rs-myexample");
    ImGui_ImplGlfw_Init(app, false);

    // rs-multicam
    rs2::context                ctx;
    rs2::colorizer              colorizer;
    std::vector<rs2::pipeline>  pipelines;
    texture                     depth_image;

    std::size_t                 dev_id = 0;
    auto         list = ctx.query_devices();
    for (auto&& dev : list)
    {
        sync(&dev, dev_id);
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        dev_id++;
    }

    // rs-record

    bool recorded = false;
    bool recording = false;
    int seek_pos;

    std::vector<std::thread> threads;
    while (app)
    {
        init(&app);
        std::vector<view> new_views;

        for (auto &&pipe : pipelines)
        {
            rs2::device device = pipe.get_active_profile().get_device();
            std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

            if (!device.as<rs2::playback>()) {
                view viewer;
                viewer.serial_num = serial;
                viewer.frame = pipe.wait_for_frames();
                new_views.emplace_back(viewer);
            }
        }
        button();
        for (auto &&pipe : pipelines)
        {
            rs2::device device = pipe.get_active_profile().get_device();
            if (!device.as<rs2::playback>()) {
                ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 90});
                ImGui::Text("Click 'record' to start recording");
                ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 110 });
                if (ImGui::Button("record", { 50, 50 }))
                {
                    if (!device.as<rs2::recorder>())
                    {
                        pipe.stop();
                        rs2::pipeline pipe;
                        rs2::config cfg; // Declare a new configuration
                        cfg.enable_record_to_file("a.bag");
                        pipe.start(cfg); //File will be opened at this point
                        device = pipe.get_active_profile().get_device();
                    }
                    else{
                        device.as<rs2::recorder>().resume();
                    }
                    recording = true;
                }
            }
            if (device.as<rs2::recorder>())
            {
                if (recording){
                    ImGui::SetCursorPos({ app.width() / 2 - 100, 3 * app.height() / 5 + 60 });
                    ImGui::TextColored({ 255 / 255.f, 64 / 255.f, 54 / 255.f, 1 }, "Recording to file 'a.bag'");
                }
                ImGui::SetCursorPos({ app.width() / 2, 3 * app.height() / 5 + 110 });
                if (ImGui::Button("pause\nrecord", { 50, 50 }))
                {
                    device.as<rs2::recorder>().pause();
                    recording = false;
                }

                ImGui::SetCursorPos({ app.width() / 2 + 100, 3 * app.height() / 5 + 110 });
                if (ImGui::Button(" stop\nrecord", { 50, 50 }))
                {
                    pipe.stop(); // Stop the pipeline that holds the file and the recorder
                    rs2::pipeline pipe; //Reset the shared pointer with a new pipeline
                    pipe.start(); // Resume streaming with default configuration
                    device = pipe.get_active_profile().get_device();
                    recorded = true; // Now we can run the file
                    recording = false;
                }
            }

            if (recorded) {
                play(app, device, &pipe);
            }

            if (device.as<rs2::playback>())
            {
                pause(app,device,pipe,seek_pos);
            }
        }
        ImGui::PopStyleColor(4);
        ImGui::PopStyleVar();

        ImGui::End();
        ImGui::Render();

        rs2::frame depth;
        float w = app.width() ;
        float h = app.height() ;
        int counter = 0;
        for (const auto& viewer : new_views)
        {
            float scale = counter * 0.25f;
            depth = colorizer.process(viewer.frame.get_depth_frame());
            //cout serial-num & timestamp
            std::cout<<viewer.serial_num<<' '<<depth.get_timestamp()<<'\n';
            depth_image.render(depth, {150 +  w * ( 0.25f - scale) , 500 - h * 0.25f , 200 + w * ( 0.5f - scale) ,  720 - h * (0.75f)});
            counter += 1;
            counter %= list.size();
        }
        for(auto &t:threads){
            t.join();
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void sync(rs2::device *dev, int dev_id) {
    auto advanced_dev = dev->as<rs400::advanced_mode>();
    auto advanced_sensors = advanced_dev.query_sensors();

    bool depth_found = false;
    bool color_found = false;

    rs2::sensor depth_sensor;
    rs2::sensor color_sensor;

    for (auto&& sensor : advanced_sensors) {
        std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
        if (module_name == "Stereo Module") {
            depth_sensor = sensor;
            depth_found = true;
        }
        else if (module_name == "RGB Camera") {
            color_sensor = sensor;
            color_found = true;
        }
    }

    if (dev_id == 0) {
        std::cout << "Setting " << dev_id << " to master!" << std::endl;
        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 1);
    } else {
        std::cout << "Setting " << dev_id << " to slave!" << std::endl;
        depth_sensor.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, 2);
    }
}

void init (window *app){
    ImGui_ImplGlfw_NewFrame(1);
    ImGui::SetNextWindowSize({ app->width(), app->height() });
    ImGui::Begin("app", nullptr, flags);
}

void button () {
    ImGui::PushStyleColor(ImGuiCol_TextSelectedBg, { 1, 1, 1, 1 });
    ImGui::PushStyleColor(ImGuiCol_Button, { 36 / 255.f, 44 / 255.f, 51 / 255.f, 1 });
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, { 36 / 255.f, 44 / 255.f, 51 / 255.f, 1 });
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);

}

void play (window app,rs2::device device,rs2::pipeline *pipe) {
    ImGui::SetCursorPos({ app.width() / 2 - 100, 4 * app.height() / 5 + 30 });
    ImGui::Text("Click 'play' to start playing");
    ImGui::SetCursorPos({ app.width() / 2 - 100, 4 * app.height() / 5 + 50});
    if (ImGui::Button("play", { 50, 50 }))
    {
        if (!device.as<rs2::playback>())
        {
            pipe->stop(); // Stop streaming with default configuration
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_device_from_file("a.bag");
            pipe.start(cfg); //File will be opened in read mode at this point
            device = pipe.get_active_profile().get_device();
        }
        else
        {
            device.as<rs2::playback>().resume();
        }
    }
}

void pause (window app,rs2::device device, rs2::pipeline &pipe, int seek_pos) {
    rs2::playback playback = device.as<rs2::playback>();
    /*if (pipe->poll_for_frames(&frames)) // Check if new frames are ready
    {
        depth = color_map.process(frames.get_depth_frame()); // Find and colorize the depth data for rendering
    }*/

    // Render a seek bar for the player
    float2 location = { app.width() / 4, 4 * app.height() / 5 + 110 };
    draw_seek_bar(playback , &seek_pos, location, app.width() / 2);

    ImGui::SetCursorPos({ app.width() / 2, 4 * app.height() / 5 + 50 });
    if (ImGui::Button(" pause\nplaying", { 50, 50 }))
    {
        playback.pause();
    }

    ImGui::SetCursorPos({ app.width() / 2 + 100, 4 * app.height() / 5 + 50 });
    if (ImGui::Button("  stop\nplaying", { 50, 50 }))
    {
        pipe.stop();
        rs2::pipeline pipe;
        pipe.start();
        device = pipe.get_active_profile().get_device();
    }
}

void draw_seek_bar(rs2::playback& playback, int* seek_pos, float2& location, float width)
{
    int64_t playback_total_duration = playback.get_duration().count();
    auto progress = playback.get_position();
    double part = (1.0 * progress) / playback_total_duration;
    *seek_pos = static_cast<int>(std::max(0.0, std::min(part, 1.0)) * 100);
    auto playback_status = playback.current_status();
    ImGui::PushItemWidth(width);
    ImGui::SetCursorPos({ location.x, location.y });
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
    if (ImGui::SliderInt("##seek bar", seek_pos, 0, 100, "", true))
    {
        //Seek was dragged
        if (playback_status != RS2_PLAYBACK_STATUS_STOPPED) //Ignore seek when playback is stopped
        {
            auto duration_db = std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(playback.get_duration());
            auto single_percent = duration_db.count() / 100;
            auto seek_time = std::chrono::duration<double, std::nano>((*seek_pos) * single_percent);
            playback.seek(std::chrono::duration_cast<std::chrono::nanoseconds>(seek_time));
        }
    }
    std::string time_elapsed = pretty_time(std::chrono::nanoseconds(progress));
    ImGui::SetCursorPos({ location.x + width + 10, location.y });
    ImGui::Text("%s", time_elapsed.c_str());
    ImGui::PopStyleVar();
    ImGui::PopItemWidth();
}

std::string pretty_time(std::chrono::nanoseconds duration)
{
    using namespace std::chrono;
    auto hhh = duration_cast<hours>(duration);
    duration -= hhh;
    auto mm = duration_cast<minutes>(duration);
    duration -= mm;
    auto ss = duration_cast<seconds>(duration);
    duration -= ss;
    auto ms = duration_cast<milliseconds>(duration);

    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(hhh.count() >= 10 ? 2 : 1) << hhh.count() << ':' <<
        std::setfill('0') << std::setw(2) << mm.count() << ':' <<
        std::setfill('0') << std::setw(2) << ss.count();
    return stream.str();
}
