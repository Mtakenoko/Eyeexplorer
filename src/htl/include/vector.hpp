#ifndef VECTOR_CALC_HPP_
#define VECTOR_CALC_HPP_

#include <vector>
namespace htl
{
    class Vector
    {
    public:
        template <class T>
        T static mean(std::vector<T> v);
        template <class T>
        T static median(std::vector<T> v);
    };

    template <class T>
    T Vector::mean(std::vector<T> v)
    {
        size_t size = v.size();
        T sum = 0;
        for (size_t i = 0; i < size; i++)
        {
            sum += v[i];
        }
        return sum / size;
    }

    template <class T>
    T Vector::median(std::vector<T> v)
    {
        size_t size = v.size();
        std::vector<T> _v(v.size());
        std::copy(v.begin(), v.end(), _v.begin());
        T tmp;
        for (size_t i = 0; i < size - 1; i++)
        {
            for (size_t j = i + 1; j < size; j++)
            {
                if (_v[i] > _v[j])
                {
                    tmp = _v[i];
                    _v[i] = _v[j];
                    _v[j] = tmp;
                }
            }
        }
        if (size % 2 == 1)
        {
            return _v[(size - 1) / 2];
        }
        else
        {
            return (_v[(size / 2) - 1] + _v[size / 2]) / 2;
        }
    }
} // namespace htl

#endif