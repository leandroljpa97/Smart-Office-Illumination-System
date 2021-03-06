#include "node.h"

using namespace std;

Node::Node() {
  rho = 0;

  y[0] = 0;
  y[1] = 0;

  n = 0;
  m = 0;

  cost_best = 0;

  c = 0;
  o = 0;
  L = 0;
  index = 0;
}                                                                                                                                                                                                                                                                                          

Node::Node( float _rho, int _c) {
  rho = _rho;

  n = 0;
  m = 0;

  cost_best = 10000.0;

  c = _c;
  o = 0;
  L = 0;
  index = 0;
}                                                                                                                                                                                                                                                                                          

void Node::consensus_init( int _nr_nos ) {
  for (int i = 0; i < _nr_nos; i++)
  {
    d_av[i] = 0;
    y[i] = 0;
  }
}

uint8_t Node::check_feasibility( Vector <float>& _d_solut, Vector <float>& _k ) {

  float tol = 0.001;

  if ( _d_solut[index] < (0 - tol) )
    return false;
  if ( _d_solut[index] > (100 + tol) )
    return false;
  if ( ( _d_solut * _k ) < ( L - o - tol ) )
    return false;

  return true;
}

float Node::evaluate_cost(Vector <float>& _d_solut) {

  Vector <float> aux = _d_solut - d_av;

  return ( ( c * _d_solut[index] ) + ( y * aux ) + ( ( rho / 2 ) * aux.quad_norm() ) );
}

void Node::updateBestCost( Vector <float>& _d_solut, Vector <float>& _k ) {

  Serial.print("check_feasibility: ");
  Serial.println(check_feasibility(_d_solut, _k));

  if ( check_feasibility( _d_solut, _k ) )
  {
    float cost_sol = evaluate_cost( _d_solut );
    Serial.print("cost_sol: ");
    Serial.println(cost_sol);
    if ( cost_sol < cost_best )
    {
      //tem que se fazer uma função que faça isto..
      d = _d_solut;

      //d.floatToUint(_d_solut);

      cost_best = cost_sol;
    }
  }
}

void Node::Primal_solve( Vector <float>& _k ) {

  cost_best = 10000.0;
  z = d_av * rho - y;

  z[index] = z[index] - c;
  /*
    Serial.print("z[0]: ");
    Serial.println(z[0]);
    Serial.print("z[1]: ");
    Serial.println(z[1]);
  */

  Serial.print("index: ");
  Serial.println(index);
  Serial.print("rho: ");
  Serial.println(rho);
  Serial.print("z[0]: ");
  Serial.println(z[0]);
  Serial.print("z[1]: ");
  Serial.println(z[1]);
  Serial.print("_k[0]: ");
  Serial.println(_k[0]);
  Serial.print("_k[1]: ");
  Serial.println(_k[1]);
  Serial.print("n: ");
  Serial.println(n);
  Serial.print("m: ");
  Serial.println(m);
  Serial.print("o: ");
  Serial.println(o);
  Serial.print("L: ");
  Serial.println(L);

  Serial.print("d_av[0]: ");
  Serial.println(d_av[0]);
  Serial.print("d_av[1]: ");
  Serial.println(d_av[1]);

  Serial.print("y[0]: ");
  Serial.println(y[0]);
  Serial.print("y[1]: ");
  Serial.println(y[1]);




  //--------------- unconstrained minimum ---------------
  Serial.println("--------------- sol_unconstrained ---------------");

  d_solut = z * ( 1 / rho );

  updateBestCost(d_solut, _k);
  /*
    Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  //--------------- compute minimum constrained to linear boundary ---------------
  Serial.println("--------------- sol_linear_boundary ---------------");

  d_solut = ( z * ( 1 / rho ) ) - ( ( _k * (1 / n) ) * ( o - L + ( ( 1 / rho ) * ( _k * z ) ) ) );

  updateBestCost(d_solut, _k);

  /* Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  //--------------- compute minimum constrained to 0 boundary ---------------
  Serial.println("--------------- sol_boundary_0 ---------------");

  d_solut = z * ( 1 / rho );
  d_solut[index] = 0;

  updateBestCost(d_solut, _k);
  /*
    Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  //--------------- compute minimum constrained to 100 boundary ---------------
  Serial.println("--------------- sol_boundary_100 ---------------");

  d_solut = z * ( 1 / rho );
  d_solut[index] = 100;

  updateBestCost(d_solut, _k);
  /*
    Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  //--------------- compute minimum constrained to linear and 0 boundary ---------------
  Serial.println("--------------- sol_linear_0 ---------------");

  d_solut = ( ( z * ( 1 / rho ) ) - ( _k * ( ( 1 / m ) * ( o - L ) ) ) + ( _k * ( ( 1 / rho ) * ( 1 / m ) * ( ( _k[index] * z[index] ) - ( z * _k ) ) ) ) );
  d_solut[index] = 0;

  updateBestCost(d_solut, _k);
  /*
    Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  //--------------- compute minimum constrained to linear and 100 boundary ---------------
  Serial.println("--------------- sol_linear_100 ---------------");

  d_solut = ( ( z * ( 1 / rho ) ) - ( ( _k * (o - L + ( 100 * _k[index] ) ) ) * ( 1 / m ) ) + ( _k * ( ( 1 / rho ) * ( 1 / m ) * ( ( _k[index] * z[index] ) - ( z * _k ) ) ) ) );
  d_solut[index] = 100;

  updateBestCost(d_solut, _k);

  /*Serial.print("d_solut[0]: ");
    Serial.println(d_solut[0]);
    Serial.print("d_solut[1]: ");
    Serial.println(d_solut[1]); */

  Serial.println("--------------- CONSENSUS SOLUTION ---------------");

  Serial.print("d[0]: ");
  Serial.println(d[0]);
  Serial.print("d[1]: ");
  Serial.println(d[1]);
  Serial.print("cost_best: ");
  Serial.println(cost_best);

  /*
    Serial.print("rho: ");
    Serial.println(rho);
    Serial.print("z[0]: ");
    Serial.println(z[0]);
    Serial.print("z[1]: ");
    Serial.println(z[1]);
    Serial.print("_k[0]: ");
    Serial.println(_k[0]);
    Serial.print("_k[1]: ");
    Serial.println(_k[1]);
    Serial.print("n: ");
    Serial.println(n);
    Serial.print("m: ");
    Serial.println(m);
    Serial.print("o: ");
    Serial.println(o);
    Serial.print("L: ");
    Serial.println(L);
  */

}
