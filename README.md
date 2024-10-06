# RMV-Task03
## 能量机关的识别与拟合
### **识别**
- 1.灰度化可以将3通道图像转化为单通道图像，以便进行二值化门限分割；去噪可以有效剔除图像中的异常独立噪点；二值化是为轮廓查找函数提供单通道图像
- 2.实现识别:注意到需要识别的R标和锤子与宝剑最显著差异即轮廓面积，通过提取轮廓判定面积来识别出R标和锤子，为了实现在锤头中心点画上圆点，通过内外轮廓父子关系提取出锤头的长方形轮廓，最后通过计算中心点的坐标画上圆点。避免遍历次数过多设置判定条件，找到目标就退出循环。 
> (轮廓检测参考blog) https://blog.csdn.net/iracer/article/details/90695914?ops_request_misc=&request_id=&biz_id=102&utm_term=opencv%E8%BD%AE%E5%BB%93%E8%AF%86%E5%88%AB&utm_medium=distribute.pc_search_result.none-task-blog-2~blog~sobaiduweb~default-2-90695914.nonecase&spm=1018.2226.3001.4450

>  ![circle](https://github.com/user-attachments/assets/d62daeec-bd6f-45e2-95ed-cab2b644f40d)

### **拟合**
**配置环境**
- 安装 eigen3:1.sudo apt-get update 2.sudo apt-get install libeigen3-dev
- 编译安装ceres1.14时出现的问题： 
  - 1.找不到TBB库中的 tbb—stddef.h 尝试重新安装TBB 仍然报错 尝试修改文件中的CMakeList.txt查找TBB并且连接 仍然报错  修改find——package参数无效  手动指定路径无效  查询后发现可能是stddef.h已在新版本删除  检查FindTBB.cmake移除stddef.h修改为version.h仍然是同样的问题  尝试关闭查找tbb库无效
  - 2.尝试重新安装，在编译安装ceres前安装前置eigen3时修改头文件路径  可以进行安装 在make过程中遇到报错  这是Eigen版本的问题，需要安装Eigen3.2.5版本，重新在/urs目录下安装3.2.5版本的Eigen：将Eigen3.2.5安装成功。在ceres-solver/cmake下将FindEigen.cmake文件替换
- 尝试运行程序报错：
   -  1.找不到ceres  在CMakeLists中配置搜索ceres的路径
   -  2.  main函数找不到windhill.hpp   添加 include 目录到项目的包含路径中
**代码部分**
- 1.要计算角速度，首先使用角速度 想通过角度创建计算角速度的函数记录在残差函数构建残差函数 最后发现拟合效果不佳
- 2.使用角度进行构建残差函数，第一次通过两帧中点角度的变化记录，拟合效果不好 后利用识别的两点形成的向量的单位向量x方向投影记录cos值 代入函数进行拟合
- 3.设置参数的上下界进行减少迭代的次数，同时设置5%收敛条件及时退出循环
- 4.配置优化选项，可以设置迭代次数的限制
- 5.最后注释掉显示图像的部分
