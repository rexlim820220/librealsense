#include <example.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <pcl/filters/passthrough.h>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

struct state {
    state() {
        yaw      =  0,0;
        pitch    =  0,0;
        last_x   =  0.0;
        last_y   =  0.0;
        ml       =false;
        offset_x = 0.0f;
        offset_y = 0.0f;
    }
    bool ml;
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    float offset_x;
    float offset_y;

};

float3 colors[] { { 0.8f, 0.1f, 0.3f },
                  { 0.1f, 0.9f, 0.5f },
                };

pcl_ptr points_to_pcl(const rs2::points& points);
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

int main(int argc, char **argv) {
    rs2::config cfg;
    cfg.enable_device_from_file("/media/adam/Transcend/20130603/D415/20190603_163655.bag");
    rs2::pipeline pipe;
    pipe.start(cfg);

    window app(1280, 720 , "PCL Example");
    state app_state;
    register_glfw_callbacks(app, app_state);

    rs2::pointcloud pc;
    rs2::points point;
    rs2::frameset frames;
    std::vector<pcl_ptr> layers;

    if (pipe.poll_for_frames(&frames)) {
        rs2::frame depth = frames.get_depth_frame();
        point = pc.calculate(depth);
        auto pcl_points = points_to_pcl(point);
        pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0,1.0);
        pass.filter(*cloud_filtered);

        layers.push_back(pcl_points);
        layers.push_back(cloud_filtered);
        pcl::io::savePCDFile("/home/adam/Desktop/3655.pcd", *pcl_points, false);
    }

    /*
    while(app) {
        //draw_pointcloud(app, app_state, layers);
    }
    */

    return EXIT_SUCCESS;
}

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}


void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
                //std::cout<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}
