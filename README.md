## 用软体模拟解迷宫可达性问题

曾经发过一个概念视频，表明软体模拟可以用来查找二维迷宫中的路径：https://www.bilibili.com/video/BV1wU4y1r7zZ/

但显然，计算速度是一个问号。**这种方法是否可以到达和BFS相似的速度？**

于是就有了本仓库。在**判断可达性**上，基本可以回答是**Yes**，而且**本身不用到BFS搜索算法，完全依靠物理和几何性质**。不过提取目标路径的过程我还没有想到如何避免部分或全部对铰链进行BFS/DFS遍历的办法。

本仓库的代码可以通过[洛谷B3625迷宫寻路](https://www.luogu.com.cn/problem/B3625)。

### 说明

目前的输入输出格式参考前述题目页的描述。

另外，`visual.hpp`包括了可视化的代码，本地运行时将渲染模拟过程中的结果。提交OJ时仅提交`simulation-maze.cpp`会自动关闭所有可视化内容。

### 可调整参数

`simulation-maze.cpp`的`23`到`25`行有三个参数：
+ `timestep`: 模拟的时间步长，越小模拟越精确。
+ `n_iters`：Kaczmarz算法迭代次数，越大模拟越精确。
+ `rand_force`：对所有质点添加随机扰动，更容易从几何上判定不可达情况。

OJ提交时时间步为`1.0`，迭代次数为`1`，这是非常粗糙、非常糟糕的模拟精度，模拟结果和现实物理相去甚远。但是，可达性判定的正确性并不依赖于最终位置精确与否。

观察模拟过程可以设置时间步为`0.001`至`0.01`，迭代次数为`50`至`500`，并关闭随机扰动。这是比较正常的渲染用模拟的质量。如果需要非常精确地进行工程计算，那么本仓库的代码并不适用——
本仓库代码采用了近似处理来保持每个模拟步骤的复杂度都不超过线性于迷宫内格子数量。

后面还有两个参数`timeout`和`dont_stop`，是为了探索和调试需要，可以设置超时时间（默认1秒）和已经得到可达性结果也不要停止模拟，可用来研究可视化模拟过程。
