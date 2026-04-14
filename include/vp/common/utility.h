//
// Created by original source on 2021/7/9.
//

#ifndef Utility_H
#define Utility_H

namespace vp::math {
    const double epsilon = 1e-6;

    inline int factorial(int x) {
        int fac = 1;
        for (int i = x; i > 0; i--)
            fac = fac * i;
        return fac;
    };
    inline int get_binomial_coefficients(int n, int k) {
        if (k == 0 || k == n)
            return 1;
        return get_binomial_coefficients(n - 1, k - 1) + get_binomial_coefficients(n - 1, k);
    };  // factorial(n)/(factorial(k)*factorial(n-k))
}  // namespace vp::math
#endif  // Utility_H
