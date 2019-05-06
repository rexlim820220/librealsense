# rs-myexample Sample

## Overview

The project we're embarking on aims to create a program that simultaneously records depth frames from two (or even more) distinct D435 cameras.

## Code Overview

The project we're embarking on aims to create a program that simultaneously records depth frames from two (or even more) distinct D435 cameras. 

| Required Info                         |                                                                |
|---------------------------------|------------------------------------------- |
| Camera Model                       | RealSense D400 series | 
| Firmware Version                   | 05.08.15.00 | 
| Operating System & Version |  Linux Mint  | 
| Kernel Version (Linux Only)    |  4.16.18                                         | 
| Platform                                 | PC |
| SDK Version                            | v2.10.0                         | 
| Language                            |  C++                          |  

![232178](https://user-images.githubusercontent.com/13239005/56457595-c3634400-63af-11e9-9aed-ef6dffad3a3d.jpg)

Before setting up, we had our camera hardware synchronized such that the recorded frames have aligned frame numbers. Since the viewer can't invoke the record streaming of two channels immediately at the time, we had no choice but to craft one-from scratch. In short, what we're doing is to combine the functions of `multicam` and `record-play-back` in SDK's example together, we also referred to https://github.com/IntelRealSense/librealsense/issues/2637 sample code to achieve hardware synchronization. 

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/13239005/56457789-500f0180-63b2-11e9-8ca2-83691859784c.gif)


Firstly, two depth frames seems to race each others for the bandwidth resources. As can be observed from my screenshot, since when one camera  is set to master and another is set to slave, one of their frame rate would be delayed. The only one reasonable solution we can coming up is allow multi-threading work during the streaming.  
`
    std::size_t              dev_id = 0;
    rs2::context                    ctx;
    rs2::colorizer            colorizer;
    std::vector<std::thread>    threads;
    texture                 depth_image;
    auto     list = ctx.query_devices();
    for (auto&& dev : list)
    {
            threads.emplace_back([dev, dev_id](){
                sync(&dev, dev_id);
                rs2::pipeline pipe(ctx);
                rs2::config cfg;
                cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
                pipe.start(cfg);
                dev_id++;
            }
    }
`
However, we are coming across the problem that how to access the former pipeline with its frames stored in `std::vector` of `std::thread` in the first loop. The stored frames would be reached and being rendered with the following snippet of codes, inside the `while (app)` part that alert the GUI window. 
`
    for (const auto& viewer : new_views)
    {
            float scale = counter * 0.25f;
            depth = colorizer.process(viewer.frame.get_depth_frame());
            depth_image.render(depth, {150 +  w * ( 0.25f - scale) , 500 - h * 0.25f , 200 + w * ( 0.5f - 
            scale) ,  720 - h * (0.75f)});
            counter += 1;
            counter %= list.size();
     }
`
The `counter` variable aims to distinguish the channel that GUI's `texture` render in `"example.hpp"` from different cameras. So the problem is stuck at how to achieve an efficiently thread managing as stated before. We had tried searching and it was not straightforward to get this particular solution from the google results (without the benefit of hindsight that is). 
The second issue happens when the `ImGui::Button("record")` is clicked, the program shut down automatically. As stated before, most of our executing code was revised from that in `multicam` and `record-play-back`. Especially the GUI control interface in `record-play-back` is really complicated, we have no idea what to do next; so far, we are wondering that if there is any substitution for the viewer to starting streaming two cameras or anyone could give us some tips on revising the code.