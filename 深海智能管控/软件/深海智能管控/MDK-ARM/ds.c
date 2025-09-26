#include <stdio.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "ds.h"      // 假设这是ds.c的头文件
#include <string.h>  // 添加这个头文件来声明 strstr 函数
#include <stdlib.h>
#include <stdbool.h>  
#include <stdint.h>


extern UART_HandleTypeDef huart3;


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_CLASSES 4
#define NUM_FEATURES 12

extern volatile float ds_sensor_data[NUM_FEATURES];

// 假设的模型参数
float svm_weights[NUM_CLASSES][NUM_FEATURES] = {{0.04049941, 0.00306499, -0.0381526, 0.00392874, -0.0045223, -0.00351625, 0.0136795, -0.40725002, -0.52075168, -0.19689864, -0.02370614, 0.0885493}, 
{0.53833936, -0.18782536, 0.55032872, -0.22501062, 0.05340414, -0.00637233, -0.27714763, 0.0321853, 0.06730994, -0.51840679, -0.07012091, -0.07304121}, 
{-1.26315413, 0.54312873, -1.27518681, 0.3938654, -0.12744198, 0.24275727, -1.13211979, 1.09914116, 1.41034243, 0.05453043, 0.05423489, -0.24973413}, 
{1.03523732, 0.40443999, 1.12982398, 0.64306366, 0.15841112, -0.20288303, 1.94220636, 0.23558726, -0.1694061, 1.33255989, 0.10232141, 0.50678159}};
float svm_bias[NUM_CLASSES] = {-0.56820803, -0.66761817, -2.25108658, -1.08191421};

float naive_bayes_means[NUM_CLASSES][NUM_FEATURES] = {{4.08428571e-01, 2.70000000e+01, 4.17928571e-01, 2.80000000e+01, 2.29078571e+00, -2.30278571e+00, -1.25167357e+02, 1.10092857e+03, 1.61716429e+03, 2.32585714e+03, 2.76230000e+03, 2.35822857e+03}, 
		{4.72642857e-01, 1.20857143e+01, 4.89071429e-01, 1.20857143e+01, 5.30907143e+00, -4.00321429e+00, -1.26099286e+02, 3.43780714e+03, 3.52222143e+03, 2.15670714e+03, 2.75613571e+03, 2.25948571e+03},
		{4.25785714e-01, 2.70000000e+01, 4.30214286e-01, 2.75642857e+01, -8.93357143e-01, -2.02721429e+00, -1.26839357e+02, 3.39510000e+03, 3.53390714e+03, 2.76395000e+03, 2.78068571e+03, 2.31701429e+03}, 
    {4.44714286e-01, 2.60000000e+01, 4.53357143e-01, 2.66642857e+01, 1.80642857e-01, -3.92642857e-01, -1.17056143e+02, 3.53760714e+03, 3.46525714e+03, 3.31546429e+03, 2.98457857e+03, 2.35515000e+03}};

float naive_bayes_variances[NUM_CLASSES][NUM_FEATURES] = {{1.14963340e-03, 1.08495993e-03, 1.15709768e-03, 1.08495993e-03, 1.62339308e+01, 1.34538065e+01, 2.32730530e+01, 1.28596853e+05, 1.88789241e+04, 5.55602353e+03, 7.65394210e+00, 1.69005983e+02}, 
		{1.12440380e-03, 5.19508809e+01, 1.16052625e-03, 5.19508809e+01, 4.99069420e+01, 1.51351682e+01, 1.09820516e+01, 1.89234246e+03, 7.98591972e+01, 2.83316532e+03, 1.19326666e+01, 1.50822231e+03}, 
		{1.10934258e-03, 1.08495993e-03, 1.10705687e-03, 2.46952307e-01, 5.12008215e+01, 3.24939279e+01, 3.39194669e+00, 3.32359197e+04, 1.18656748e+02, 8.23854858e+03, 3.99165952e+01, 4.59686595e+02},
    {1.10987829e-03, 1.08495993e-03, 1.13154666e-03, 2.24095164e-01, 4.86261240e+00, 7.81722583e+00, 3.60581294e+00, 2.68492532e+03, 6.08577820e+02, 3.53526021e+05, 6.68269592e+04, 2.86271442e+02}};
float naive_bayes_class_priors[NUM_CLASSES] = {0.25, 0.25, 0.25, 0.25};

float min_value[NUM_FEATURES] = {0, 0, 0, 0, -180, -180, -180, 0, 0, 0, 0, 0};  // 示例最小值
float max_value[NUM_FEATURES] = {100, 50, 100, 50, 180, 180, 180, 4095, 4095, 4095, 4095, 4095};  // 示例最大

void normalization(float *probabilities, int length) {
    float sum = 0.0;
    for (int i = 0; i < length; i++) {
        sum += probabilities[i];
    }

    if (sum > 0.0f) {
        for (int i = 0; i < length; i++) {
            probabilities[i] /= sum;
        }
    }
}

void calculate_svm_prob(float *features, float *probabilities) {
    float max_score = -INFINITY;

    // 计算原始分数（logits），并找到最大值以便数值稳定
    for (int i = 0; i < NUM_CLASSES; i++) {
        probabilities[i] = svm_bias[i];
        for (int j = 0; j < NUM_FEATURES; j++) {
            probabilities[i] += svm_weights[i][j] * features[j];
        }
        if (probabilities[i] > max_score) {
            max_score = probabilities[i];  // 找到最大logit值
        }
    }

    // 计算softmax分母和中间结果，减去最大值以提高数值稳定性
    float sum_exp = 0.0f;
    for (int i = 0; i < NUM_CLASSES; i++) {
        probabilities[i] = exp(probabilities[i] - max_score);  // 减去最大值进行归一化
        sum_exp += probabilities[i];
    }

    // 将分数归一化为概率
    for (int i = 0; i < NUM_CLASSES; i++) {
        probabilities[i] /= sum_exp;
    }
}

void calculate_naive_bayes_prob(float *features, float *probabilities) {
    // 先进行特征的归一化，确保数值范围一致
    for (int i = 0; i < NUM_FEATURES; i++) {
        features[i] = (features[i] - min_value[i]) / (max_value[i] - min_value[i]);
    }

    // 打印每个类别的均值和方差
    for (int i = 0; i < NUM_CLASSES; i++) {
//        printf("Class %d Means: ", i);
        for (int j = 0; j < NUM_FEATURES; j++) {
//            printf("%.6f ", naive_bayes_means[i][j]);
        }
//        printf("\nClass %d Variances: ", i);
        for (int j = 0; j < NUM_FEATURES; j++) {
//            printf("%.6f ", naive_bayes_variances[i][j]);
        }
//        printf("\n");
    }

    // 开始朴素贝叶斯的概率计算
    for (int i = 0; i < NUM_CLASSES; i++) {
        // 初始概率为先验概率
        probabilities[i] = logf(naive_bayes_class_priors[i]);

        for (int j = 0; j < NUM_FEATURES; j++) {
            float variance = naive_bayes_variances[i][j];
            float mean = naive_bayes_means[i][j];
            float feature = features[j];

            // 防止方差为零，进行拉普拉斯平滑处理
            if (variance < 1e-6f) {
                variance = 1e-6f;
            }

            // 平滑处理，避免某些特征值的影响过大
            float smoothing_factor = 1e-3f;

            float log_term = logf(2 * M_PI * variance);
            float exp_term = ((feature - mean) * (feature - mean)) / (2 * variance);

            probabilities[i] -= (0.5f * log_term + exp_term + smoothing_factor);
        }

        // 限制 exp() 输入范围，避免下溢出
        if (probabilities[i] > 100) probabilities[i] = 100;
        else if (probabilities[i] < -100) probabilities[i] = -100;

        probabilities[i] = expf(probabilities[i]);  // 计算最终概率
    }

    // 归一化处理，确保总概率为1
    normalization(probabilities, NUM_CLASSES);
}




void calculate_decision_tree_prob(float *features, float *probabilities) {
    // 根据提供的决策树规则进行分类
    // 假设 features[0] 是 'a'，features[8] 是 'i'，features[10] 是 'k'
    
    if (features[0] <= 0.44f) {  // 'a' <= 0.44
        if (features[8] <= 2936.00f) {  // 'i' <= 2936.00
            probabilities[0] = 1.0;  // 类别 0
            probabilities[1] = 0.0;
            probabilities[2] = 0.0;
            probabilities[3] = 0.0;
        } else {  // 'i' > 2936.00
            probabilities[0] = 0.0;
            probabilities[1] = 0.0;
            probabilities[2] = 1.0;  // 类别 2
            probabilities[3] = 0.0;
        }
    } else {  // 'a' > 0.44
        if (features[10] <= 2769.00f) {  // 'k' <= 2769.00
            probabilities[0] = 0.0;
            probabilities[1] = 1.0;  // 类别 1
            probabilities[2] = 0.0;
            probabilities[3] = 0.0;
        } else {  // 'k' > 2769.00
            probabilities[0] = 0.0;
            probabilities[1] = 0.0;
            probabilities[2] = 0.0;
            probabilities[3] = 1.0;  // 类别 3
        }
    }
}

void ds_fusion(float *svm_prob, float *naive_bayes_prob, float *decision_tree_prob, float *fused_prob) {
    // 将每个分类器的权重设为相等，1:1:1 分配
    float equal_weight = 1.0f / 3.0f; // 每个分类器的权重

    // 初始化融合概率数组
    for (int i = 0; i < NUM_CLASSES; i++) {
        // 进行加权求和，此处权重相等
        fused_prob[i] = equal_weight * svm_prob[i] + 
                        equal_weight * naive_bayes_prob[i] + 
                        equal_weight * decision_tree_prob[i];
    }

    // 打印融合后的概率
    // printf("Fused probabilities (weighted sum): [%.2f, %.2f, %.2f, %.2f]\n", fused_prob[0], fused_prob[1], fused_prob[2], fused_prob[3]);

    // 归一化处理，确保总概率为 1
    float sum = 0.0f;
    for (int i = 0; i < NUM_CLASSES; i++) {
        sum += fused_prob[i];
    }

    if (sum > 0.0f) {
        for (int i = 0; i < NUM_CLASSES; i++) {
            fused_prob[i] /= sum;
        }
    }

    // 打印归一化后的融合概率
    printf("Normalized fused probabilities: [%.2f, %.2f, %.2f, %.2f]\n", fused_prob[0], fused_prob[1], fused_prob[2], fused_prob[3]);
}







