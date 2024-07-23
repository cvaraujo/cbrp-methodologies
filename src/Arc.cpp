//
// Created by carlos on 06/03/19.
//

#include "../headers/Arc.h"

Arc::Arc(int o, int d, int length, int block)
{
    this->o = o;
    this->d = d;
    this->length = length;
    this->block = block;
}

int Arc::getO() const
{
    return o;
}

int Arc::getD() const
{
    return d;
}

int Arc::getLength() const
{
    return length;
}

int Arc::getBlock() const
{
    return block;
}

void Arc::setBlock(int block)
{
    this->block = block;
}

void Arc::setO(int o)
{
    this->o = o;
}

void Arc::setD(int d)
{
    this->d = d;
}