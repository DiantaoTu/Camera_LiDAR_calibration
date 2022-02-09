/*
 * @Author: Diantao Tu
 * @Date: 2021-11-19 14:03:13
 */

#include "../include/FileIO.h"

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

bool ExportMatchPair(const std::string folder, const std::vector<MatchPair>& pairs)
{
    if(folder.empty())
        return false;
    LOG(INFO) << "Save match pair in " << folder;
    // 保存文件之前要先把之前的都删了
    if(boost::filesystem::exists(folder))
        boost::filesystem::remove_all(folder);
    boost::filesystem::create_directory(folder);

    for(int i = 0; i < pairs.size(); i++)
    {
        ofstream out(folder + "/" + int2str(i) + ".bin");
        boost::archive::binary_oarchive out_archive(out);
        out_archive << pairs[i];
        out.close();
    }
    return true;
}

bool ReadMatchPair(const std::string folder, std::vector<MatchPair>& pairs)
{
    omp_set_num_threads(11);
    LOG(INFO) << "Loading match pair from " << folder;
    if(!boost::filesystem::exists(folder))
        return false;
    vector<string> names;
    IterateFiles(folder, names, ".bin");
    if(names.empty())
        return false;
    pairs.clear();
    #pragma omp parallel for
    for(size_t i = 0; i < names.size(); i++)
    {
        ifstream in(names[i]);
        boost::archive::binary_iarchive in_archive(in);
        MatchPair pair;
        in_archive >> pair;
        in.close();
        #pragma omp critical
        {
            pairs.push_back(pair);
        }
    }
    // 经过上面openmp的并行操作后，image_pair的顺序就被打乱了，重新按图像的索引排序，排列成
    // 0-1  0-2  0-3  0-4 ... 1-2  1-3  1-4 ... 2-3  2-4  ... 3-4 这样的顺序
    // 注意：这种方式只能排序10000张图像
    sort(pairs.begin(), pairs.end(), 
        [](const MatchPair& mp1,const MatchPair& mp2)
        {return 10000 * mp1.image_pair.first + mp1.image_pair.second < 10000 * mp2.image_pair.first + mp2.image_pair.second;}
        );
    return true;
}

bool ExportFrame(const std::string folder, const std::vector<Frame>& frames)
{
    if(folder.empty())
        return false;
    for(const Frame& f : frames)
    {
        // 从 /aaa/bbb/ccc/xxx.jpg 变成 xxx.bin
        string name = f.name;
        name = name.substr(name.find_last_of("/") + 1);
        name = name.substr(0, name.size() - 4);
        name += ".bin";
        ofstream out(folder + "/" + name);
        boost::archive::binary_oarchive archive_out(out);
        archive_out << f;
        out.close();
    }
    return true;
}

bool ReadFrames(const std::string frame_folder, const std::string image_folder, std::vector<Frame>& frames)
{
    vector<string> image_names, frame_names;
    IterateFiles(image_folder, image_names, ".jpg");
    IterateFiles(frame_folder, frame_names, ".bin");
    if(image_names.size() != frame_names.size())
    {
        LOG(ERROR) << "num of images != num of frames";
        return false;
    }
    bool success = true;
    // 这里目前假设所有图像都是相同的尺寸
    cv::Mat img = cv::imread(image_names[0]);
    omp_set_num_threads(11);
    #pragma omp parallel for schedule(dynamic)
    for(int i = 0; i < image_names.size(); i++)
    {
        Frame frame(img.rows, img.cols, i, image_names[i]);
        // 记录一下初始的id和路径名，和读取后的对比，要求二者一致才行
        int old_id = frame.id;
        string old_name = frame.name;
        ifstream input(frame_names[i]);
        boost::archive::binary_iarchive ia(input);
        ia >> frame;
        // 这里只检查了图像的id是否一致，没有检查图像的路径名，因为可能存在一种情况是在 A电脑上跑了结果，然后
        // 又把结果放到了B电脑上继续跑，那这种情况下路径就不一样了，但是实际上图像是一样的，所以就不检查路径了。
        // 然而id需要一致，如果不一致就说明是图像顺序变了或者是数量变了等等
        // 同样检查了图像的行列数，保证是相同的图片
        if(frame.id != old_id || frame.GetImageRows() != img.rows || frame.GetImageCols() != img.cols)
        {
            success = false;
        }
        frame.name = old_name;
        if(frame.GetDescriptor().rows == 0)
        {
            LOG(WARNING) << "no descriptor in frame, may cause error in image matching"; 
        }

        #pragma omp critical 
        {
            frames.push_back(frame);
        }
    }
    // 对frame按id大小排列，因为经过openmp乱序执行
    sort(frames.begin(), frames.end(), [](Frame& a, Frame& b){return a.id < b.id;});
    if(success)
        LOG(INFO) << "Load " << frames.size() << " images";
    else 
        LOG(INFO) << "Load images failed";
    return success;    
}
