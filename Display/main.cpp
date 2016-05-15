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

pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer( "3D Display" ) ); // 点云的显示 viewer

const string file_name = "Display.ini"; // 配置文件名称

vector<AnimateCloud> cloud_list; // 动态点云列表

vector<vector<AnimateStatus> > script_list; // 剧本, 记录每一帧每个点云的AnimateStatus

vector<vector<string> > tag_list;                   // 每一帧的说明文字
vector<vector<float> > tag_size_list;               // 每一帧的说明文字的大小
vector<vector<pcl::PointXYZ> > tag_position_list;   // 每一帧的说明文字的位置

vector<vector<cv::Mat> > image_list;    // 每一帧用于辅助显示的2D图片
vector<string> frame_reg_names;         // 每一帧显示的2D图片使用不同的imshow窗口名称

// 播放参数 
int curr_frame = 0,     // 当前帧
    curr_image = 0,     // 当前图片
    curr_cnt_tags = 0,  // 当前帧的标签总数
    cnt_cloud,  // 动态点云总数
    cnt_frame;  // 帧总数

// 载入演员 cloud_list
// 载入剧本 script_list
// 载入说明文字 tag_list
// 载入辅助图片 image_list !
void PrepareScene()
{
    cout<< "开始加载文件" << file_name << endl;
    ifstream script_file(file_name);
    if (!script_file.is_open()) {
        cout<< "ifstream读取失败, 请检查文件" << file_name << endl;
        system("pause");
    }

    // load cloud by file names
    while (true) {
        string cloud_file_name;
        script_file >> cloud_file_name;
        if (cloud_file_name=="---") break;

        // 读取pointcloud并生成AnimateCloud
        stringstream ss; ss<<"cloud"<<cnt_cloud;
        cout << "正在加载点云["<<cnt_cloud<<"]"<<cloud_file_name<<endl;

        PointCloudPtr new_cloud(new PointCloud);
        pcl::io::loadPCDFile<PointT>(cloud_file_name, *new_cloud);
        AnimateCloud anim_cloud(new_cloud, ss.str());
        cloud_list.push_back(anim_cloud);
        
        ++cnt_cloud;
    }


    // load script
    cout<<"开始加载动画信息"<<endl;
    while (true) {
        // load script_list
        vector<AnimateStatus> list;
        
        string line; char c;
        script_file >> line;
        if (line=="---") break; else c=line[0];

        for (int j=0; j<cnt_cloud; ++j) {
            while (!isalpha(c) && !script_file.eof()) script_file>>c;
            switch (c) {
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
                cout << "警告: 在加载第"<<cnt_frame<<"帧第"<<j<<"个点云信息时出现未知字符\'"<<c<<"\'("<<(int)c<<"), 请检查配置文件."<< endl;
                list.push_back(AnimateStatus::Stop);
                break;
            }

            c = ' ';
        }
        script_list.push_back(list);

        // load tags   (+)
        // load images (*)
        vector<string> tags; 
        vector<pcl::PointXYZ> tags_position;
        vector<float> tags_size;
        vector<cv::Mat> images;
        while (true) {
            script_file >> line;
            
            if (line[0] == '+') {
                pcl::PointXYZ pt;
                float sz;
                char chrs[100];
                script_file >> pt.x >> pt.y >> pt.z >> sz;
                script_file.getline(chrs, 100);

                string str = chrs;
                tags.push_back(str);
                tags_position.push_back(pt);
                tags_size.push_back(sz);
            } else if (line[0] == '*') {
                script_file >> line;
                cv::Mat img = cv::imread(line);
                if (img.empty())
                    cout<< "警告:　无法加载辅助图片"<<line<<endl;
                else 
                    images.push_back(img);
            } else break;
        }
        tag_list.push_back(tags);
        tag_position_list.push_back(tags_position);
        tag_size_list.push_back(tags_size);
        image_list.push_back(images);

        // 每一帧显示的2D图片使用不同的imshow窗口名称 
        stringstream ss; ss<<"frame"<<cnt_frame;
        frame_reg_names.push_back(ss.str());

        cout<<"第"<<cnt_frame<<"帧加载完成. "<<tags.size()<<"个说明标签, "<<images.size()<<"个辅助图片."<<endl;

        ++cnt_frame;
    }

    cout<<"加载工作已完成!"<<endl;
}

/* 键盘响应 */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    if (event.getKeyCode() == ']' && event.keyDown()) {
        if (curr_frame + 1 < cnt_frame) {
            ++curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "第"<<curr_frame<<"帧" << endl;
        } else {
            cout << "已经是最后一帧了." << endl;
        }
    }

    if (event.getKeyCode() == '[' && event.keyDown()) {
        if (curr_frame > 0) {
            --curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "第"<<curr_frame<<"帧" << endl;
        } else {
            cout << "已经是第零帧了." << endl;
        }
    }

    // 清理和绘制说明文字
    for (int i=0; i<curr_cnt_tags; ++i) {
        stringstream ss; ss<<"text"<<i; 
        viewer->removeText3D(ss.str());
    }
    curr_cnt_tags = tag_list[curr_frame].size();
    for (int i=0; i<curr_cnt_tags; ++i) {
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

    while (!viewer->wasStopped()) {
        // 刷新viewer
        viewer->spinOnce(40);

        // 更新动态点云状态
        for (int i=0; i<cnt_cloud; ++i) cloud_list[i].Refresh();
        
        // 更新辅助图片的显示
        if (image_list[curr_frame].size()!=0) {
            ++curr_image;
            curr_image %= image_list[curr_frame].size();
            cv::imshow(frame_reg_names[curr_frame], image_list[curr_frame][curr_image]);
        }

        // 检查点云和viewer的关联
        for (int i=0; i<cnt_cloud; ++i) 
            if (cloud_list[i].isVisible()) {
                if( !viewer->updatePointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() ) )
                    viewer->addPointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() );
            } else {
                viewer->removePointCloud( cloud_list[i].getRegName() );
            }
    }
}