#include <iostream>
#include <algorithm>
#include <vector>

const double MAX_DIST = 4.0;
const double INPUT_MAX = 1.0;
const double INPUT_MIN = 0.0;
const int kNumSamples = 100;  // サンプル数
double dist_min = MAX_DIST/4;
double dist_mid = MAX_DIST/2;
double dist_max = MAX_DIST*3/4;

class FuzzyInference{
  public:
    double short_distance(double d) {
      if (d < dist_min) {
        return INPUT_MAX;
      } else if (d < dist_mid) {
        return dist_mid - d;
      } else {
        return INPUT_MIN;
      }
    }

    double medium_distance(double d) {
      if (d < dist_min || d > dist_max) {
        return INPUT_MIN;
      } if (d < dist_mid) {
        return d - dist_min;
      } else { // d < dist_max
        return dist_max - d;
      }
    }

    double long_distance(double d) {
      if (d < dist_mid) {
        return INPUT_MIN;
      } else if (d < dist_max) {
        return d - dist_mid;
      } else {
        return INPUT_MAX;
      }
    }

    double apply_rule1(double short_distance) {
      return short_distance;
    }

    double apply_rule2(double medium_distance) {
      return medium_distance;
    }

    double apply_rule3(double long_distance) {
      return long_distance;
    }

    double defuzzify(double rule1_output, double rule2_output, double rule3_output) {
      // 重心計算のために、各ルールの貢献度を計算する
      double w1 = 0.0;
      double w2 = 1.0;
      double w3 = 2.0;

      double numerator = rule1_output * w1 + rule2_output * w2 + rule3_output * w3;
      double denominator = rule1_output + rule2_output + rule3_output;

      // 重心を計算して、それを出力とする
      double defuzzify_output = numerator / denominator;

      return defuzzify_output/w3;
    }
};


int main() {
  FuzzyInference fuzzy_inference;

  double d;
  std::cout << "Enter distance: ";
  std::cin >> d;

  // 3つのメンバーシップ関数を呼び出す
  double short_distance = fuzzy_inference.short_distance(d);
  double medium_distance = fuzzy_inference.medium_distance(d);
  double long_distance = fuzzy_inference.long_distance(d);

  std::cout << "short_dist: " << short_distance << std::endl;
  std::cout << "medium_dist: " << medium_distance << std::endl;
  std::cout << "long_dist: " << long_distance << std::endl;
  
  // 3つのルールを適用する
  double rule1 = fuzzy_inference.apply_rule1(short_distance);
  double rule2 = fuzzy_inference.apply_rule2(medium_distance);
  double rule3 = fuzzy_inference.apply_rule3(long_distance);

  // 脱ファジィ化を行う
  double output = fuzzy_inference.defuzzify(rule1, rule2, rule3);
  std::cout << "Output: " << output << std::endl;
  return 0;
}

