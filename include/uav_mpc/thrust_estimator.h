#ifndef THRUST_ESTIMATOR_H
#define THRUST_ESTIMATOR_H

#include <queue>
#include <utility>
#include <cmath>
#include <algorithm>

class ThrustEstimator {
private:
    // 历史油门队列，存储 <时间戳(秒), 油门百分比(0~1)>
    std::queue<std::pair<double, double>> timed_thrust_; 
    
    // RLS 算法状态参数
    const double rho2_ = 0.998; // 遗忘因子
    double thr2acc_;            // 待估计的推力映射系数 (a_z = thr * thr2acc_)
    double P_;                  // 协方差矩阵 (一阶系统中为标量)
    
    // 延迟补偿时间窗口
    const double delay_max_ = 0.050; // 50ms
    const double delay_min_ = 0.020; // 20ms

public:
    /**
     * @brief 构造函数，初始化估计器
     * @param hover_percentage 经验悬停油门百分比 (例如 0.58 表示 58% 油门悬停)
     * @param gravity 当地重力加速度 (例如 9.8066)
     */
    ThrustEstimator(double hover_percentage, double gravity) {
        if (hover_percentage <= 0.05) hover_percentage = 0.58;
        thr2acc_ = gravity / hover_percentage; // 初始猜测值 (如 9.8066 / 0.58 = 16.9)
        P_ = 50.0;                             // 适度初始协方差，防止首个样本导致数值发散
    }

    /**
     * @brief 在每次向底层下发油门指令时调用，记录历史数据
     * @param current_time_s 当前时间(秒)
     * @param thrust         下发的油门百分比(0~1)
     */
    void pushThrustRecord(double current_time_s, double thrust) {
        timed_thrust_.push({current_time_s, thrust});
        
        // 限制队列长度，防止内存无限增长
        while (timed_thrust_.size() > 100) {
            timed_thrust_.pop();
        }
    }

    /**
     * @brief 在控制器的循环中高频调用，在线更新映射系数
     * @param current_time_s 当前时间(秒)
     * @param real_acc_z     IMU当前测量的 Z 轴实际加速度 (m/s^2)
     * @return true表示本次更新了参数，false表示无合适数据更新
     */
    bool estimateThrustModel(double current_time_s, double real_acc_z) {
        // 过滤非物理加速度噪声
        if (std::isnan(real_acc_z) || real_acc_z < 2.0 || real_acc_z > 30.0) {
            return false;
        }

        while (timed_thrust_.size() >= 1) {
            auto t_t = timed_thrust_.front();
            double time_passed = current_time_s - t_t.first;

            if (time_passed > delay_max_) {
                // 数据太旧，已错过匹配窗口，丢弃
                timed_thrust_.pop();
                continue;
            }
            if (time_passed < delay_min_) {
                // 队首数据还太新，电机的响应还没完全体现在当前加速度上，退出等待
                return false;
            }

            // 找到了延迟窗口内匹配的油门数据
            double thr = t_t.second;
            timed_thrust_.pop();

            if (thr < 0.15 || thr > 0.95) {
                return false;
            }

            // ---------------------------------------------------------
            // 带有遗忘因子的 RLS (递归最小二乘) 算法核心
            // ---------------------------------------------------------
            double denom = rho2_ + thr * P_ * thr;
            if (denom < 1e-4) denom = 1e-4;
            double K = P_ * thr / denom;
            
            // 预测误差 = 实际加速度 - 预测加速度(thr * thr2acc_)
            double error = real_acc_z - thr * thr2acc_;
            thr2acc_ = thr2acc_ + K * error;
            
            // 更新协方差并实施下界保护
            P_ = (1.0 - K * thr) * P_ / rho2_;
            P_ = std::max(1e-4, std::min(500.0, P_));

            // 物理边界硬约束：限制悬停油门在 [0.28, 0.85] 对应 thr2acc_ in [11.5, 35.0]
            thr2acc_ = std::max(11.5, std::min(35.0, thr2acc_));

            return true;
        }
        return false;
    }

    /**
     * @brief 利用估计出的系数，计算出为了产生期望加速度所需的油门量
     * @param desired_acc_z 期望的Z轴加速度 (m/s^2)
     * @return 实际应该下发给底层的油门百分比 (0.15 ~ 0.92)
     */
    double computeDesiredThrust(double desired_acc_z) {
        if (std::isnan(thr2acc_) || thr2acc_ < 10.0) {
            thr2acc_ = 16.9;
        }
        double thr = desired_acc_z / thr2acc_;
        if (std::isnan(thr)) {
            thr = 0.58;
        }
        return std::max(0.15, std::min(0.92, thr));
    }
    
    // 获取当前估计出的映射系数 (用于调试或日志记录)
    double getThr2Acc() const { return thr2acc_; }
};

#endif
