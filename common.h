/*
 * @Author: Diantao Tu
 * @Date: 2021-10-11 16:28:02
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <boost/filesystem.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

using namespace std;

template<typename T>
using eigen_vector = std::vector<T, Eigen::aligned_allocator<T>>;
template<typename Key, typename Value>
using eigen_map = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<Key, Value>>>;


// 递归地遍历某个文件夹下的文件，并选出以 filtType 为结尾的文件
void IterateFiles(string pathName, vector<string> &fileNames, string fileType);

// convert a int to a string
string int2str(int num) ;

int str2int(string str) ;

string float2str(float num);

float str2float(string str);

string double2str(double num);

double str2double(string str) ;

string long2str(long num) ;

bool FileNameComp(string a, string b);


#endif