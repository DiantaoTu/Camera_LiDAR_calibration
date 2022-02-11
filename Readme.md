# Camera LiDAR Calibration

这是对论文 [Automatic Online Calibration of Cameras and Lasers](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.309.7957&rep=rep1&type=pdf) 的复现

受于水平所限，复现的结果无法达到论文原作者的效果(在KITTI上测试)

注意以下几点：

1. 论文中对图像进行距离逆变换(inverse distance transform) 时，作者称该方法的时间复杂度是 O(n)，但我只能实现 O(n^2) 的算法，所以导致速度很慢。因此增加了保存变换后的图像到本地的功能，方便多次使用。
2. 论文中的公式4是通过Fc来判断当前标定是正确标定的概率。但是这个公式在Fc=0时取得最小值0.437，在Fc=1.002时取得最大值0.5。因此我个人判断这个公式有问题，就没有使用。