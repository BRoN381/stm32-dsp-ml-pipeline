/**
 * @file inference.h
 * @brief TFLite inference engine wrapper.
 * Expected to handle model loading, tensor allocation, and gesture classification.
 */
#ifndef INFERENCE_H
#define INFERENCE_H

void Inference_Init(void);
int Inference_Classify(float* features);

#endif // INFERENCE_H
