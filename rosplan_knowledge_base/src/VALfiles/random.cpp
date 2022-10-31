/************************************************************************
 * Copyright 2008, Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 *
 * Maria Fox, Richard Howey and Derek Long - VAL
 * Stephen Cresswell - PDDL Parser
 *
 * This file is part of VAL, the PDDL validator.
 *
 * VAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * VAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with VAL.  If not, see <http://www.gnu.org/licenses/>.
 *
 ************************************************************************/

/*-----------------------------------------------------------------------------
  VAL - The Automatic Plan Validator for PDDL+

  $Date: 2009-02-05 10:50:27 $
  $Revision: 1.2 $

  Maria Fox, Richard Howey and Derek Long - PDDL+ and VAL
  Stephen Cresswell - PDDL Parser

  maria.fox@cis.strath.ac.uk
  derek.long@cis.strath.ac.uk
  stephen.cresswell@cis.strath.ac.uk
  richard.howey@cis.strath.ac.uk

  By releasing this code we imply no warranty as to its reliability
  and its use is entirely at your own risk.

  Strathclyde Planning Group
  http://planning.cis.strath.ac.uk
 ----------------------------------------------------------------------------*/
#include "random.h"

namespace VAL1_2 {
  
NormalGen Generators::randomNumberNormGenerator = NormalGen();
UniformGen Generators::randomNumberUniGenerator = UniformGen(0,0,1);

//return a random number with norm prob over -1 to 1
double getRandomNumberNormal()
{
     double randomNumber;
     do
     {
       randomNumber = Generators::randomNumberNormGenerator()*0.25;
     }while(randomNumber > 1.0 || randomNumber < -1.0);

     //cout << randomNumber << " \\\\\n";
     return randomNumber;
};

//return a random number with uniform prob over 0 to 1
double getRandomNumberUniform()
{
     //double randomNumber = double(rand()) / double(RAND_MAX);
     double randomNumber = Generators::randomNumberUniGenerator();

     return randomNumber;
};

double getRandomNumberPsuedoNormal()
{
  
  int noToAverage = 4;
  double total = 0;

  for(int i = 1; i <= noToAverage; ++i)
  {
     total += getRandomNumberUniform();
  };

  return total/noToAverage;
};

};
