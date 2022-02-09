/*
 * @Author: Diantao Tu
 * @Date: 2021-11-19 14:03:13
 */

#include "FileIO.h"

using namespace std;

vector<string> SplitString(const string& str, const char& delimma)
{
    vector<string> split;
    stringstream ss(str);
    string tmp;
    while (getline(ss, tmp, delimma))
    {
        split.push_back(tmp);
    }
    return split;
}

void ReadPoseT(std::string file_path, eigen_vector<Eigen::Matrix3d>& rotation_list, 
            eigen_vector<Eigen::Vector3d>& trans_list, std::vector<std::string>& name_list)
{
    ifstream in(file_path);
    if(!in.is_open())
    {
        LOG(ERROR) << "Fail to open " << file_path << endl;
        return;
    }

    while (!in.eof())
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        Eigen::Vector3d t = Eigen::Vector3d::Ones() * numeric_limits<double>::infinity();

        string str;
        getline(in, str);
        vector<string> sub_strings = SplitString(str, ' ')  ;
        
        // 如果分割出13个子串，那就说明是含有名字的，就把第一个子串作为名字保存下来
        if(sub_strings.size() == 13)
        {
            name_list.push_back(sub_strings[0]);
            sub_strings.erase(sub_strings.begin());
        }
        // 如果有12个子串，就说明恰好是R t
        if(sub_strings.size() == 12)
        {
            for(const string& s : sub_strings)
            {
                if(s.find("inf") != string::npos || s.find("nan") != string::npos)
                {
                    goto end;
                }
            }
            R(0,0) = str2double(sub_strings[0]);
            R(0,1) = str2double(sub_strings[1]);
            R(0,2) = str2double(sub_strings[2]);
            t.x() = str2double(sub_strings[3]);
            R(1,0) = str2double(sub_strings[4]);
            R(1,1) = str2double(sub_strings[5]);
            R(1,2) = str2double(sub_strings[6]);
            t.y() = str2double(sub_strings[7]);
            R(2,0) = str2double(sub_strings[8]);
            R(2,1) = str2double(sub_strings[9]);
            R(2,2) = str2double(sub_strings[10]);
            t.z() = str2double(sub_strings[11]);
        }
        end:
        rotation_list.push_back(R);
        trans_list.push_back(t);
        if(in.peek() == EOF)
            break;
    }
    in.close();
    return;
}

void ReadPoseQt(std::string file_path, std::vector<Eigen::Matrix3d>& rotation_list, 
            std::vector<Eigen::Vector3d>& trans_list, std::vector<std::string>& name_list)
{
    ifstream in(file_path);
    if(!in.is_open())
    {
        LOG(ERROR) << "Fail to open " << file_path << endl;
        return;
    }
    while (!in.eof())
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
        Eigen::Vector3d t = Eigen::Vector3d::Ones() * numeric_limits<double>::infinity();

        string str;
        getline(in, str);
        vector<string> sub_strings = SplitString(str, ' ')  ;
        
        // 如果分割出8个子串，那就说明是含有名字的，就把第一个子串作为名字保存下来
        if(sub_strings.size() == 8)
        {
            name_list.push_back(sub_strings[0]);
            sub_strings.erase(sub_strings.begin());
        }
        // 如果有7个子串，就说明恰好是R t
        if(sub_strings.size() == 7)
        {
            for(const string& s : sub_strings)
            {
                if(s.find("inf") != string::npos || s.find("nan") != string::npos)
                {
                    goto end;
                }
            }
            double qx, qy, qz, qw;
            qx = str2double(sub_strings[0]);
            qy = str2double(sub_strings[1]);
            qz = str2double(sub_strings[2]);
            qw = str2double(sub_strings[3]);
            t.x() = str2double(sub_strings[4]);
            t.y() = str2double(sub_strings[5]);
            t.z() = str2double(sub_strings[6]);
            R = Eigen::Matrix3d(Eigen::Quaterniond(qw, qx, qy, qz));
        }
        end:
        rotation_list.push_back(R);
        trans_list.push_back(t);
        if(in.peek() == EOF)
            break;
    }
    in.close();
    return;
}

void ExportPoseT(const std::string file_path, const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& rotation_list,
                const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& trans_list,
                const std::vector<string>& name_list)
{
    ofstream out(file_path);
    if(!out.is_open())
    {
        LOG(ERROR) << "Fail to write " << file_path << endl;
        return;
    }
    assert(rotation_list.size() == trans_list.size());
    for(size_t i = 0; i < rotation_list.size(); i++)
    {
        if(i < name_list.size())
            out << name_list[i] << " ";
        const Eigen::Matrix3d& R_wc = rotation_list[i];
        const Eigen::Vector3d& t_wc = trans_list[i];
        out << R_wc(0,0) << " " << R_wc(0,1) << " " << R_wc(0,2) << " " << t_wc(0) << " "
            << R_wc(1,0) << " " << R_wc(1,1) << " " << R_wc(1,2) << " " << t_wc(1) << " "
            << R_wc(2,0) << " " << R_wc(2,1) << " " << R_wc(2,2) << " " << t_wc(2) << endl;
    }
    out.close();
    return;
}



