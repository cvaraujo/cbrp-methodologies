//
// Created by Carlos on 06/07/2021.
//

#ifndef DPARP_ARC_H
#define DPARP_ARC_H

class Arc
{

private:
  int o, d, block, length;

public:
  Arc(int o, int d, int length, int block)
  {
    this->o = o;
    this->d = d;
    this->length = length;
    this->block = block;
  }

  int getO() { return o; }

  int getD() { return d; }

  int getLength() { return length; }

  int getBlock() { return block; }

  void setBlock(int block) { this->block = block; }

  void setLength(int length) { this->length = length; }

  void setO(int o) { this->o = o; }

  void setD(int d) { this->d = d; }
};

#endif // MRP_ARC_H
