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
        new pcl::visualization::PCLVisualizer( "3D Display" ) ); // ���Ƶ���ʾ viewer

const string file_name = "Display.ini"; // �����ļ�����

vector<AnimateCloud> cloud_list; // ��̬�����б�

vector<vector<AnimateStatus> > script_list; // �籾, ��¼ÿһ֡ÿ�����Ƶ�AnimateStatus

vector<vector<string> > tag_list;                   // ÿһ֡��˵������
vector<vector<float> > tag_size_list;               // ÿһ֡��˵�����ֵĴ�С
vector<vector<pcl::PointXYZ> > tag_position_list;   // ÿһ֡��˵�����ֵ�λ��

vector<vector<cv::Mat> > image_list;    // ÿһ֡���ڸ�����ʾ��2DͼƬ
vector<string> frame_reg_names;         // ÿһ֡��ʾ��2DͼƬʹ�ò�ͬ��imshow��������

// ���Ų��� 
int curr_frame = 0,     // ��ǰ֡
    curr_image = 0,     // ��ǰͼƬ
    curr_cnt_tags = 0,  // ��ǰ֡�ı�ǩ����
    cnt_cloud,  // ��̬��������
    cnt_frame;  // ֡����

// ������Ա cloud_list
// ����籾 script_list
// ����˵������ tag_list
// ���븨��ͼƬ image_list !
void PrepareScene()
{
    cout<< "��ʼ�����ļ�" << file_name << endl;
    ifstream script_file(file_name);
    if (!script_file.is_open()) {
        cout<< "ifstream��ȡʧ��, �����ļ�" << file_name << endl;
        system("pause");
    }

    // load cloud by file names
    while (true) {
        string cloud_file_name;
        script_file >> cloud_file_name;
        if (cloud_file_name=="---") break;

        // ��ȡpointcloud������AnimateCloud
        stringstream ss; ss<<"cloud"<<cnt_cloud;
        cout << "���ڼ��ص���["<<cnt_cloud<<"]"<<cloud_file_name<<endl;

        PointCloudPtr new_cloud(new PointCloud);
        pcl::io::loadPCDFile<PointT>(cloud_file_name, *new_cloud);
        AnimateCloud anim_cloud(new_cloud, ss.str());
        cloud_list.push_back(anim_cloud);
        
        ++cnt_cloud;
    }


    // load script
    cout<<"��ʼ���ض�����Ϣ"<<endl;
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
                cout << "����: �ڼ��ص�"<<cnt_frame<<"֡��"<<j<<"��������Ϣʱ����δ֪�ַ�\'"<<c<<"\'("<<(int)c<<"), ���������ļ�."<< endl;
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
                    cout<< "����:���޷����ظ���ͼƬ"<<line<<endl;
                else 
                    images.push_back(img);
            } else break;
        }
        tag_list.push_back(tags);
        tag_position_list.push_back(tags_position);
        tag_size_list.push_back(tags_size);
        image_list.push_back(images);

        // ÿһ֡��ʾ��2DͼƬʹ�ò�ͬ��imshow�������� 
        stringstream ss; ss<<"frame"<<cnt_frame;
        frame_reg_names.push_back(ss.str());

        cout<<"��"<<cnt_frame<<"֡�������. "<<tags.size()<<"��˵����ǩ, "<<images.size()<<"������ͼƬ."<<endl;

        ++cnt_frame;
    }

    cout<<"���ع��������!"<<endl;
}

/* ������Ӧ */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
    if (event.getKeyCode() == ']' && event.keyDown()) {
        if (curr_frame + 1 < cnt_frame) {
            ++curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "��"<<curr_frame<<"֡" << endl;
        } else {
            cout << "�Ѿ������һ֡��." << endl;
        }
    }

    if (event.getKeyCode() == '[' && event.keyDown()) {
        if (curr_frame > 0) {
            --curr_frame; curr_image = 0;
            for (int i=0; i<cnt_cloud; ++i)
                cloud_list[i].setAnimateStatus(script_list[curr_frame][i]);
            cout << "��"<<curr_frame<<"֡" << endl;
        } else {
            cout << "�Ѿ��ǵ���֡��." << endl;
        }
    }

    // ����ͻ���˵������
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
        // ˢ��viewer
        viewer->spinOnce(40);

        // ���¶�̬����״̬
        for (int i=0; i<cnt_cloud; ++i) cloud_list[i].Refresh();
        
        // ���¸���ͼƬ����ʾ
        if (image_list[curr_frame].size()!=0) {
            ++curr_image;
            curr_image %= image_list[curr_frame].size();
            cv::imshow(frame_reg_names[curr_frame], image_list[curr_frame][curr_image]);
        }

        // �����ƺ�viewer�Ĺ���
        for (int i=0; i<cnt_cloud; ++i) 
            if (cloud_list[i].isVisible()) {
                if( !viewer->updatePointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() ) )
                    viewer->addPointCloud( cloud_list[i].getPointCloud(), cloud_list[i].getRegName() );
            } else {
                viewer->removePointCloud( cloud_list[i].getRegName() );
            }
    }
}