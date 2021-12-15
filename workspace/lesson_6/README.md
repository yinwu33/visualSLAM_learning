# Task2

简答题请见pdf

### 核心代码

```c++
// TODO START YOUR CODE HERE (~8 lines)
double error = 0;
Eigen::Vector2d J;  // Jacobian
if (inverse == false) {
    // Forward Jacobian
    double x_curr = kp.pt.x + x + dx;
    double y_curr = kp.pt.y + y + dy;
    error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) - GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
    J[0] = -0.5 * (GetPixelValue(img2, x_curr + 1, y_curr) - GetPixelValue(img2, x_curr - 1, y_curr));
    J[1] = -0.5 * (GetPixelValue(img2, x_curr, y_curr + 1) - GetPixelValue(img2, x_curr, y_curr - 1));
}
else {
    // Inverse Jacobian
    // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
    double x_curr = kp.pt.x + x;
    double y_curr = kp.pt.y + y;
    error = GetPixelValue(img2, x_curr, y_curr) - GetPixelValue(img1, x_curr - dx, y_curr - dy);

    // J[0] = -1 * (GetPixelValue(img1, x_curr + 1, y_curr) - GetPixelValue(img1, x_curr - 1, y_curr)) / 2;
    // J[1] = -1 * (GetPixelValue(img1, x_curr, y_curr + 1) - GetPixelValue(img1, x_curr, y_curr - 1)) / 2;
    J(0) = 0.5*(GetPixelValue(img1,kp.pt.x + x + 1 , kp.pt.y + y) - GetPixelValue(img1,kp.pt.x + x - 1 , kp.pt.y + y));
    J(1) = 0.5*(GetPixelValue(img1,kp.pt.x + x , kp.pt.y + y + 1) - GetPixelValue(img1,kp.pt.x + x , kp.pt.y + y - 1));
}

// compute H, b and set cost;
H += J * J.transpose();
b += -J * error;
cost += error * error / 2;
// TODO END YOUR CODE HERE

```



### 运行结果

![task2](/home/ubuntu/Data/Share/vSLAM/homework/土豆猪肉啤酒-第6章作业/task2/task2.png)

# Task3

### 核心代码

```c++
// TODO START YOUR CODE HERE
// * 1. compute 3d points
Eigen::Vector3d point3d_ref;
point3d_ref[2] = depth_ref[i];
point3d_ref[0] = (px_ref[i][0] - cx) / fx * point3d_ref[2];
point3d_ref[1] = (px_ref[i][1] - cy) / fy * point3d_ref[2];


// * 2. transform point3d
Eigen::Vector3d point3d_curr = T21 * point3d_ref;
double X = point3d_ref[0];
double Y = point3d_ref[1];
double Z = point3d_ref[2];

if (point3d_curr[2] < 0)
    continue;

// * 3. projection 3d to 2d
double u =0, v = 0;
u = fx * point3d_curr[0] / point3d_curr[2] + cx;
v = fy * point3d_curr[1] / point3d_curr[2] + cy;

// * 4. check in the area
if (u - half_patch_size < 0 or u + half_patch_size > img2.cols or
    v - half_patch_size < 0 or v + half_patch_size > img2.rows)
    continue;

nGood++;
goodProjection.push_back(Eigen::Vector2d(u, v));

// and compute error and jacobian
for (int x = -half_patch_size; x < half_patch_size; x++)
    for (int y = -half_patch_size; y < half_patch_size; y++) {

        double error = 0;
        error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) - GetPixelValue(img2, u + x, v + y);

        Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
        J_pixel_xi << fx / Z , 0 , -fx * X / Z / Z , -fx * X * Y / Z / Z , fx + fx * X * X / Z / Z, -fx * Y / Z
            , 0 , fy / Z , -fy * Y / Z / Z , -fy - fy * Y * Y / Z / Z , fy * X * Y / Z / Z , fy * X / Z;
        Eigen::Vector2d J_img_pixel;    // image gradients
        J_img_pixel[0] = static_cast<double>((GetPixelValue(img2, u + x + 1, v + y) - GetPixelValue(img2, u + x - 1, v + y)) * 0.5);
        J_img_pixel[1] = static_cast<double>((GetPixelValue(img2, u + x, v + y + 1) - GetPixelValue(img2, u + x, v + y - 1)) * 0.5);
        // total jacobian
        Vector6d J;
        J = -J_pixel_xi.transpose() * J_img_pixel;

        H += J * J.transpose();
        b += -error * J;
        cost += error * error;
    }
// END YOUR CODE HERE
```



### 运行结果

![task3](/home/ubuntu/Data/Share/vSLAM/homework/土豆猪肉啤酒-第6章作业/task3/task3.png)

# Task4

### Idea

由于disparity只考虑x方向，因此修改模型，不考虑dy，此时矩阵`update`, `H`, `b`都变成标量double

详细请见代码

### 运行结果

由terminal结果可见，关于x的平均error大概在-40左右，有可能是代码写错了

![task4](/home/ubuntu/Data/Share/vSLAM/homework/土豆猪肉啤酒-第6章作业/task4/task4.png)
