#ifndef DS_H
#define DS_H

#include <math.h>   // 用于数学计算
#include <stdint.h> // 用于标准整数类型

// 定义分类器类别数量和特征数量
#define NUM_CLASSES 4
#define NUM_FEATURES 12

// 声明全局变量
extern float svm_weights[NUM_CLASSES][NUM_FEATURES];
extern float svm_bias[NUM_CLASSES];
extern float naive_bayes_means[NUM_CLASSES][NUM_FEATURES];
extern float naive_bayes_variances[NUM_CLASSES][NUM_FEATURES];
extern float naive_bayes_class_priors[NUM_CLASSES];

// 函数声明
void normalization(float *probabilities, int length);
void parse_data_to_features(const char *data, float *features);
void ds_fusion(float *svm_prob, float *naive_bayes_prob, float *decision_tree_prob, float *fused_prob);  // 添加分号
void calculate_naive_bayes_prob(float *features, float *probabilities);
void calculate_decision_tree_prob(float *features, float *probabilities);
void output_probabilities_to_serial(float *svm_prob, float *naive_bayes_prob, float *decision_tree_prob);
void read_sensor_data(void);
void handle_uart_data(const char *uart_data); // 添加分号

#endif /* DS_H */
