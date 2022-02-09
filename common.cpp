/*
 * @Author: Diantao Tu
 * @Date: 2021-11-04 19:11:48
 */

#include "../include/common.h"

using namespace std;

// 递归地遍历某个文件夹下的文件，并选出以 filtType 为结尾的文件
void IterateFiles(string pathName, vector<string> &fileNames, string fileType)
{
    fileNames.clear();
    if(!boost::filesystem::exists(pathName))
        return;
    boost::filesystem::directory_iterator endIter;
    for (boost::filesystem::directory_iterator iter(pathName); iter != endIter; ++iter)
    {
        if (boost::filesystem::is_regular_file(iter->status()))
        {
            std::string type = iter->path().extension().string();
            transform(type.begin(), type.end(), type.begin(), (int (*)(int))tolower);
            if (type == fileType)
                fileNames.push_back(iter->path().string());
        }
        else if (boost::filesystem::is_directory(iter->path()))
        {
            cout << iter->path().string() << endl;
            vector<string> names;
            IterateFiles(iter->path().string(), names, fileType);
            fileNames.insert(fileNames.end(), names.begin(), names.end());
        }
    }
    sort(fileNames.begin(), fileNames.end(), std::less<std::string>());
}

// convert a int to a string
string int2str(int num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

int str2int(string str) {
    int d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to int" << endl;
    exit(0);
}

string float2str(float num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

float str2float(string str) {
    float d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to float" << endl;
    exit(0);
}

string double2str(double num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[double2str] - Code error" << endl;
        exit(0);
    }
}

double str2double(string str) {
    double d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to double" << endl;
    exit(0);
}

string long2str(long num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[long2str] - Code error" << endl;
        exit(0);
    }
}

bool FileNameComp(string a, string b)
{
    int pos = int(a.find_last_of('.'));
    int pos2 = pos - 1;
    while(pos2 >= 0 && a[pos2] >='0' && a[pos2] <= '9')
        pos2 -= 1;
    pos2 += 1;
    int num_a = str2int(a.substr(pos2, pos-pos2));
    pos = int(b.find_last_of('.'));
    pos2 = pos - 1;
    while(pos2 >= 0 && b[pos2] >='0' && b[pos2] <= '9')
        pos2 -= 1;
    pos2 += 1;
    int num_b = str2int(b.substr(pos2, pos-pos2));
    return num_a < num_b;


    
}