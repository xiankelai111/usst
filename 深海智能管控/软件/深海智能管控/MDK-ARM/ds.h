#ifndef DS_H
#define DS_H

#include <math.h>   // ������ѧ����
#include <stdint.h> // ���ڱ�׼��������

// ��������������������������
#define NUM_CLASSES 4
#define NUM_FEATURES 12

// ����ȫ�ֱ���
extern float svm_weights[NUM_CLASSES][NUM_FEATURES];
extern float svm_bias[NUM_CLASSES];
extern float naive_bayes_means[NUM_CLASSES][NUM_FEATURES];
extern float naive_bayes_variances[NUM_CLASSES][NUM_FEATURES];
extern float naive_bayes_class_priors[NUM_CLASSES];

// ��������
void normalization(float *probabilities, int length);
void parse_data_to_features(const char *data, float *features);
void ds_fusion(float *svm_prob, float *naive_bayes_prob, float *decision_tree_prob, float *fused_prob);  // ��ӷֺ�
void calculate_naive_bayes_prob(float *features, float *probabilities);
void calculate_decision_tree_prob(float *features, float *probabilities);
void output_probabilities_to_serial(float *svm_prob, float *naive_bayes_prob, float *decision_tree_prob);
void read_sensor_data(void);
void handle_uart_data(const char *uart_data); // ��ӷֺ�

#endif /* DS_H */
