#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "util.hpp"

template <class Tsize, class Tentry, class Tsum>
class MovAvgFilter
{
private:
    Tentry *buffer;
    Tsize bufferLength;

    Tsize entryIndex;
    Tsize fill;
    Tsum sum;

    Tentry getValueAsEntry();
    float getValueAsFloat();
    void updateInternal(Tentry value);

public:
    MovAvgFilter() = delete;

    void reset();

    MovAvgFilter(Tentry *buffer, Tsize bufferLength)
    {
        this->buffer = buffer;
        this->bufferLength = bufferLength;
        reset();
    }

    Tentry update(Tentry value)
    {
        updateInternal(value);
        return getValueAsEntry();
    }

    float updateAsFloat(Tentry value)
    {
        updateInternal(value);
        return getValueAsFloat();
    }
};

template <class Tsize, class Tentry>
class MedianFilter
{
private:
    Tentry *buffer;
    Tsize bufferLength;

    Tsize entryIndex;
    Tentry median;
    Tsize medianDuplicateCount;
    Tsize lesserValueCount;
    Tsize greaterValueCount;

    void moveMedianToNextSmallerValue();
    void moveMedianToNextGreaterValue();
    void updateMedian(Tentry newValue);
    void shiftEntryIndex();

public:
    MedianFilter() = delete;

    void reset(Tentry fillValue);

    MedianFilter(Tentry *buffer, Tsize bufferLength, Tentry fillValue)
    {
        this->buffer = buffer;
        this->bufferLength = bufferLength;
        reset(fillValue);
    }

    Tentry update(Tentry value);
};

template <class Tsize, class Tentry, class Tsum>
class CombinedFilter
{
private:
    MedianFilter<Tsize, Tentry> medianFilter;
    MovAvgFilter<Tsize, Tentry, Tsum> movAvgFilter;

public:
    CombinedFilter() = delete;

    CombinedFilter(Tentry *medianFilterBuffer, Tsize medianFilterLength, Tentry *movAvgFilterBuffer, Tsize movAvgFilterLength, Tentry fillValue = Tentry())
        : medianFilter(medianFilterBuffer, medianFilterLength, fillValue),
          movAvgFilter(movAvgFilterBuffer, movAvgFilterLength)
    {
    }

    CombinedFilter(Tentry *totalFilterBuffer, Tsize medianFilterLength, Tsize movAvgFilterLength, Tentry fillValue = Tentry())
        : medianFilter(totalFilterBuffer, medianFilterLength, fillValue),
          movAvgFilter(totalFilterBuffer + medianFilterLength, movAvgFilterLength)
    {
    }

    void reset(Tentry fillValue)
    {
        medianFilter.reset(fillValue);
        movAvgFilter.reset();
    }

    Tentry update(Tentry value)
    {
        value = medianFilter.update(value);
        return movAvgFilter.update(value);
    }

    float updateAsFloat(Tentry value)
    {
        value = medianFilter.update(value);
        return movAvgFilter.updateAsFloat(value);
    }
};

template <class Tsize, class Tentry, class Tsum>
void MovAvgFilter<Tsize, Tentry, Tsum>::reset()
{
    entryIndex = 0;
    fill = 0;
    sum = 0;
}

template <class Tsize, class Tentry, class Tsum>
Tentry MovAvgFilter<Tsize, Tentry, Tsum>::getValueAsEntry()
{
    return (Tentry)(sum / fill);
}

template <class Tsize, class Tentry, class Tsum>
float MovAvgFilter<Tsize, Tentry, Tsum>::getValueAsFloat()
{
    return (float)sum / (float)fill;
}

template <class Tsize, class Tentry, class Tsum>
void MovAvgFilter<Tsize, Tentry, Tsum>::updateInternal(Tentry value)
{
    Tentry &currBufferEntry = buffer[entryIndex];

    bool isPoppingOldEntry = entryIndex < fill;
    if (isPoppingOldEntry)
        sum -= (Tsum)currBufferEntry;

    sum += (Tsum)value;
    currBufferEntry = value;

    entryIndex++;

    if (fill < entryIndex)
        fill = entryIndex;

    if (entryIndex >= bufferLength)
        entryIndex = 0;
}

template <class Tsize, class Tentry>
void MedianFilter<Tsize, Tentry>::reset(Tentry fillValue)
{
    entryIndex = 0;
    median = fillValue;
    medianDuplicateCount = bufferLength;
    lesserValueCount = 0;
    greaterValueCount = 0;

    for (Tsize i = 0; i < bufferLength; i++)
        buffer[i] = fillValue;
}

template <class Tsize, class Tentry>
void MedianFilter<Tsize, Tentry>::moveMedianToNextSmallerValue()
{
    Tentry nextSmallerValue = numeric_limits<Tentry>::minimum();

    for (Tsize i = 0; i < bufferLength; i++)
    {
        Tentry value = buffer[i];
        if (value > nextSmallerValue && value < median)
            nextSmallerValue = value;
    }

    median = nextSmallerValue;
    greaterValueCount = medianDuplicateCount + greaterValueCount;
    medianDuplicateCount = 0;
    lesserValueCount = 0;

    for (Tsize i = 0; i < bufferLength; i++)
    {
        Tentry value = buffer[i];
        if (value == median)
            medianDuplicateCount++;
        else if (value < median)
            lesserValueCount++;
    }
}

template <class Tsize, class Tentry>
void MedianFilter<Tsize, Tentry>::moveMedianToNextGreaterValue()
{
    Tentry nextGreaterValue = numeric_limits<Tentry>::maximum();

    for (Tsize i = 0; i < bufferLength; i++)
    {
        Tentry value = buffer[i];
        if (value < nextGreaterValue && value > median)
            nextGreaterValue = value;
    }

    median = nextGreaterValue;
    lesserValueCount = medianDuplicateCount + lesserValueCount;
    medianDuplicateCount = 0;
    greaterValueCount = 0;

    for (Tsize i = 0; i < bufferLength; i++)
    {
        Tentry value = buffer[i];
        if (value == median)
            medianDuplicateCount++;
        else if (value > median)
            greaterValueCount++;
    }
}

template <class Tsize, class Tentry>
void MedianFilter<Tsize, Tentry>::updateMedian(Tentry newValue)
{
    Tentry &entryToReplace = buffer[entryIndex];
    Tentry oldValue = entryToReplace;

    if (oldValue == newValue)
        return;

    entryToReplace = newValue;

    if (oldValue == median)
        medianDuplicateCount--;
    else if (oldValue < median)
        lesserValueCount--;
    else // oldValue > median
        greaterValueCount--;

    if (newValue == median)
        medianDuplicateCount++;
    else if (newValue < median)
        lesserValueCount++;
    else // newValue > median
        greaterValueCount++;

    Tsize middle = bufferLength / 2;

    if (lesserValueCount > middle)
        moveMedianToNextSmallerValue();
    else if (bufferLength - greaterValueCount <= middle)
        moveMedianToNextGreaterValue();
}

template <class Tsize, class Tentry>
void MedianFilter<Tsize, Tentry>::shiftEntryIndex()
{
    if (++entryIndex >= bufferLength)
        entryIndex = 0;
}

template <class Tsize, class Tentry>
Tentry MedianFilter<Tsize, Tentry>::update(Tentry value)
{
    updateMedian(value);
    shiftEntryIndex();
    return median;
}
