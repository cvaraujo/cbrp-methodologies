//
// Created by Carlos on 06/07/2021.
//

#ifndef DPARP_ARC_H
#define DPARP_ARC_H

class Arc
{

private:
  int o, d, block, cases, length;

public:
  Arc(int o, int d, int length, int block);

  int getO() const;

  int getD() const;

  int getBlock() const;

  int getLength() const;

  void setBlock(int block);

  void setO(int o);

  void setD(int d);
};

#endif // MRP_ARC_H
