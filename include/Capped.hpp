/// @brief An integer with capped addition and subtraction so that its value never exceeds the limits. 
/// @tparam low The minimum value allowed to be held
/// @tparam high The maximum value allowed to be held
template<int low, int high> class CappedInt
{
public:
    CappedInt();
    CappedInt(int init_value);
    void set(int new_value);
    int get();
    int operator++(int);
    int operator--(int);
    int operator+=(const int &rhs);
    int operator-=(const int &rhs);
    int operator+(const int &rhs);
    int operator-(const int &rhs);

private:
    int value;

    int cap(int value);
};

template<int low, int high> CappedInt<low, high>::CappedInt(): value(low) {}

template<int low, int high> CappedInt<low, high>::CappedInt(int init_value)
{
    value = cap(init_value);
}

template<int low, int high> void CappedInt<low, high>::set(int new_value)
{
    value = cap(new_value);
}

template<int low, int high> int CappedInt<low, high>::get()
{
    return value;
}

template<int low, int high> int CappedInt<low, high>::operator++(int)
{
    int old = value;
    value = cap(value + 1);
    return old;
}

template<int low, int high> int CappedInt<low, high>::operator--(int)
{
    int old = value;
    value = cap(value - 1);
    return old;
}

template<int low, int high> int CappedInt<low, high>::operator+=(const int &rhs)
{
    value = cap(value + rhs);
    return value;
}

template<int low, int high> int CappedInt<low, high>::operator-=(const int &rhs)
{
    value = cap(value - rhs);
    return value;
}

template<int low, int high> inline int CappedInt<low, high>::operator+(const int &rhs)
{
    return cap(value + rhs);
}

template<int low, int high> inline int CappedInt<low, high>::operator-(const int &rhs)
{
    return cap(value - rhs);
}

template<int low, int high> int CappedInt<low, high>::cap(int value)
{
    if (value < low) {
        return low;
    } else if (value > high) {
        return high;
    } else {
        return value;
    }
}
