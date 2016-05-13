/* Created by sugar10w, 2016.5.10
 *
 * 3D display with PCL_viewer
 */

#include "common.h"

#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "animate_cloud.h"

using namespace std;

/* 点云的显示 viewer */
pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer( "3D Display" ) );

/* 动态点云列表 */
vector<AnimateCloud> cloud_list;
/* 剧本 */
vector<vector<AnimateStatus> > script_list;
/* 用于viewer的注册名 */
vector<string> reg_names_cloud;
/* 各个说明文字 */
vector<vector<string> > tag_list;
vector<vector<float> > tag_size_list;
vector<vector<pcl::PointXYZ> > tag_position_list;
/* 辅助用2D图片 */
vector<vector<cv::Mat> > image_list;
vector<string> frame_reg_names;
/* 播放参数 */
int curr_frame = 0, cnt_cloud, cnt_frame, cnt_tags = 0, curr_image = 0;


/* 载入演员(AnimateCloud), 载入剧本(AnimateStatus)! */
void PrepareScene()
{
    string file_name = "Display.ini";
    cout<< "开始加载文件" << file_name << endl;
    ifstream script_file(file_name);

    // load cloud
    while (true)
    {
        string cloud_file_name;
        PointCloudPtr new_cloud(new PointCloud);

        script_file >> cloud_file_name;
        
        if (cloud_file_name=="---") break;

        /* 生成AnimateCloud */
        stringstream ss; ss<<"cloud"<<cnt_cloud;
        cout << "正在加载点云["<<cnt_cloud<<"]"<<cloud_file_name<<endl;
        pcl::io::loadPCDFile<PointT>(cloud_file_name, *new_cloud);
        AnimateCloud anim_cloud(new_cloud, ss.str());
        cloud_list.push_back(anim_cloud);
        
        ++cnt_cloud;
    }


    // load script
    cout<<"开始加载动画信息"<<endl;
    while (true)
    {
        vector<AnimateStatus> list;
        
        string line; 
        char c;

        script_file >> line;
        if (line=="---") break; else c=line[0];
        ++cnt_frame;

        stringstream ss; ss<<"frame"<<cnt_frame;
        frame_reg_names.push_back(ss.str());

        for (int j=0; j<cnt_cloud; ++j)
        {
            while (!isalpha(c)) script_file>>c;

            switch (c)
            {
            case 'o': case 'O':
                list.push_back(AnimateStatus::Zero);
                break;
            case 'a': case 'A':
                list.push_back(AnimateStatus::Appear);
                break;
            case 'd': case 'D':
                list.push_back(AnimateStatus::Disappear);
                break;
            case 'p': case 'P':
                list.push_back(AnimateStatus::Plane);
                break;
            case 't': case 'T':
                list.push_back(AnimateStatus::Target);
                break;
            case 'c': case 'C':
                list.push_back(AnimateStatus::Counting);
                break;
            case 'b': case 'B':
                list.push_back(AnimateStatus::Bursting);
                break;
            case 'x': case 'X':
                list.push_back(AnimateStatus::Stop);
                break;
            default:
                list.push_back(AnimateStatus::Stop);
                break;
            }

            c = ' ';
        }
        script_list.push_back(list);

        //load tags
        vector<string> tags; 
        vector<pcl::PointXYZ> tags_position;
        vector<float> tags_size;
        vector<cv::Mat> images;
        while (true)
        {
            script_file >> line;
            
            if (line[0] == '+')
            {
                pcl::PointXYZ pt;
                float sz;
                char chrs[100];
                script_file >> pt.x >> pt.y >> pt.z >> sz;
                script_file.getline(chrs, 100);

                string str = chrs;
                tags.push_back(str);
                tags_position.push_back(pt);
                tags_size.push_back(sz);
            }
            else if (line[0] == '*')
            {
                script_file >> line;
                cv::Mat img = cv::imread(line);
                images.push_back(img);
            }
            else break;
        }
        tag_list.push_back(tags);
        tag_position_list.push_back(tags_position);
        tag_size_list.push_back(tags_size);
        image_list.push_back(images);
    }

    cout<<"加载工作已完成!"<<endl;
}

/* 键盘响应 */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    if (event.getKeyCode() == ']' && event.keyDown())
    {
        if (curr_frame + 1 < cnt_frame)
        {
            ++curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "第"<<curr_frame<<"帧" << endl;
        }
        else
        {
            cout << "已经是最后一帧了." << endl;
        }
    }

    if (event.getKeyCode() == '[' && event.keyDown())
    {
        if (curr_frame > 0)
        {
            --curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "第"<<curr_frame<<"帧" << endl;
        }
        else
        {
            cout << "已经是第一帧了." << endl;
        }
    }

    for (int i=0; i<cnt_tags; ++i)
    {
        stringstream ss; ss<<"text"<<i; 
        viewer->removeText3D(ss.str());
    }
    cnt_tags = tag_list[curr_frame].size();
    for (int i=0; i<cnt_tags; ++i)
    {
        stringstream ss; ss<<"text"<<i; 
        viewer->addText3D(
            tag_list[curr_frame][i], 
            tag_position_list[curr_frame][i],
            tag_size_list[curr_frame][i],
            1, 1, 1,
            ss.str());
    }
}

int main(int argc, char* argv[])
{
    viewer->registerKeyboardCallback(keyboardEventOccurred);

    PrepareScene();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(40);

        for (int i=0; i<cnt_cloud; ++i) cloud_list[i].Refresh();
        
        if (image_list[curr_frame].size()!=0)
        {
            ++curr_image;
            curr_image %= image_list[curr_frame].size();
            cv::imshow(frame_reg_names[curr_frame], image_list[curr_frame][curr_image]);
        }

        for (int i=0; i<cnt_cloud; ++i) 
            if (cloud_list[i].isVisible())
            {
                if( !viewer->updatePointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() ) )
                    viewer->addPointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() );
            }
            else
            {
                viewer->removePointCloud( cloud_list[i].getRegName() );
            }
    }
}