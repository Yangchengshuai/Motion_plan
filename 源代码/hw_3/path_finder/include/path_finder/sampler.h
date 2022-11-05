/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _BIAS_SAMPLER_
#define _BIAS_SAMPLER_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>

class BiasSampler
{
public:
  BiasSampler()
  {
    std::random_device rd;
    gen_ = std::mt19937_64(rd());
    uniform_rand_ = std::uniform_real_distribution<double>(0.0, 1.0);
    uniform_rand_2_=std::uniform_real_distribution<double>(-1.0, 1.0);
    normal_rand_ = std::normal_distribution<double>(0.0, 1.0);
    range_.setZero();
    origin_.setZero();
  };

  void setSamplingRange(const Eigen::Vector3d origin, const Eigen::Vector3d range)
  {
    origin_ = origin;
    range_ = range;
  }

  void samplingOnce(Eigen::Vector3d &sample)
  {
    sample[0] = uniform_rand_(gen_);
    sample[1] = uniform_rand_(gen_);
    sample[2] = uniform_rand_(gen_);
    sample.array() *= range_.array();
    sample += origin_;
  };

/*单位求的采样
1. 初始化一个-1到1之间的均匀分布生成器
2. 对各个维度进行均匀采样
3. 判断是否在单位球内
*/
  bool sampleUnitNBall(Eigen::Vector3d& sample)
  {
     sample[0]=uniform_rand_2_(gen_);
     sample[1]=uniform_rand_2_(gen_);
     sample[2]=uniform_rand_2_(gen_);

     if (sample.norm()<=1)
     {
       return true;
     }else
     {
      return false;
     }     
  }


  // 初始化一些必要参数一些固定的参数只需计算一次:椭圆焦点x_f1,x_f2,旋转矩阵C
  std::uniform_real_distribution<double> uniform_rand_2_;
  Eigen::VectorXd x_f1,x_f2,x_center,a1,I_1T;;
  Eigen::MatrixXd C;
  double c_min,c_max;//c_max为已经搜索出的最优路径大小
  void setEpllise(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
  {
    x_f1=start;
    x_f2=goal;
    c_min=(x_f2-x_f1).norm();
    x_center=0.5*(x_f1+x_f2);
    a1=(x_f2-x_f1)/c_min;
    I_1T<<1,0,0; //单位矩阵第一列的转置
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a1*I_1T,Eigen::ComputeThinU|Eigen::ComputeThinV);
    Eigen::Matrix3d U=svd.matrixU();
    Eigen::Matrix3d V=svd.matrixV();
    Eigen::DiagonalMatrix<double,3> d; //对角矩阵创建
    d.diagonal()<<1,1,U.determinant()*V.determinant(); //对角矩阵赋值
    C=U*d*V.transpose(); //求出旋转矩阵C
  }
  
//c_max为已经搜索出的最优路径大小
  void informedSampling(Eigen::Vector3d &sample,double c_max)
  {
    Eigen::DiagonalMatrix<double,3> L;
    L.diagonal()<<0.5*c_max,0.5*(sqrt(c_max*c_max-c_min*c_min));

    Eigen::Vector3d x_ball;
    //满足球采样，则继续，否则不断采样
    while(!sampleUnitNBall(x_ball)){};
    sample=C*L*x_ball+x_center;
  }

  // (0.0 - 1.0)
  double getUniRandNum()
  {
    return uniform_rand_(gen_);
  }

private:
  Eigen::Vector3d range_, origin_;
  std::mt19937_64 gen_;
  std::uniform_real_distribution<double> uniform_rand_;
  
  std::normal_distribution<double> normal_rand_;
};

#endif
