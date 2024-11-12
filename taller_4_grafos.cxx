#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include "Grafo.cxx"

// TODO # 0: Incluir el archivo cabecera del grafo
// #include "Grafo.h"

// -------------------------------------------------------------------------
struct Punto
{
  float X, Y;
  float distanciaA( const Punto& b ) const
    {
      float x = X - b.X;
      float y = Y - b.Y;
      return( std::sqrt( ( x * x ) + ( y * y ) ) );
    }

  bool operator==(const Punto &p) const
  {
      return (X == p.X && Y == p.Y );
  }

};

// -------------------------------------------------------------------------

// TODO # 1: Definir el tipo para un grafo con Puntos como vertices y costos reales
// typedef Grafo< Punto, float > TGrafo 

typedef Grafo < Punto > TGrafo;
typedef std::vector< unsigned long > TRuta;
typedef std::vector< TRuta > TCaminos;
typedef std::vector< float > TDist;

// -------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
  if( argc < 2 )
  {
    std::cerr
      << "Uso: " << argv[ 0 ] << " archivo_entrada"
      << std::endl;
    return( 1 );

  }

  // TODO # 2: Declarar el grafo
    TGrafo g ;

  // Cargar el archivo en un bufer
  std::ifstream in_mesh_stream( argv[ 1 ], std::ifstream::binary );
  if( !in_mesh_stream )
  {
    std::cerr << "Error leyendo \"" << argv[ 1 ] << "\"" << std::endl;
    return( 1 );

  }
  in_mesh_stream.seekg( 0, in_mesh_stream.end );
  unsigned long in_mesh_file_length = in_mesh_stream.tellg( );
  in_mesh_stream.seekg( 0, in_mesh_stream.beg );
  char* in_mesh_file_buffer = new char[ in_mesh_file_length ];
  in_mesh_stream.read( in_mesh_file_buffer, in_mesh_file_length );
  in_mesh_stream.close( );
  std::istringstream in_mesh( in_mesh_file_buffer );

  // Leer los vertices desde el bufer
  long nPoints;
  in_mesh >> nPoints;
  for( long pId = 0; pId < nPoints; pId++ )
  {
    Punto pnt;
    in_mesh >> pnt.X >> pnt.Y;

    // TODO # 3: Insertar el Punto en el grafo
    g.insertarVertice( pnt );

  }

  // Leer las aristas desde el bufer
  long nEdges;
  in_mesh >> nEdges;
  for( long eId = 0; eId < nEdges; eId++ )
  {
    long start, end;
    in_mesh >> start >> end;

    // TODO # 4: Calcular el costo de la arista, insertarla en el grafo como no dirigida
    float cost = g.getVertices()[start].distanciaA( g.getVertices()[end] );
    g.insAristaNoDir( start, end, cost );
    

  }
  delete [] in_mesh_file_buffer;

  // Imprimir matriz de adyacencia

  std::cout<<"\n\n-------Matriz  de adyacencia del grafo -------\n";
  double ** aristas = g.getAristas();
  for (int i= 0; i<g.cantVertices();i++){

    for(int j= 0; j<g.cantVertices(); j++){
      std::cout<< aristas[i][j]<<"\t\t";
    }
    std::cout<<std::endl;
  }

  

  // TODO # 5: Calcular distancias lineales (distancia Euclidiana)
  unsigned long s_id = 0;
  float dist_s_e = 0.0;
  TDist distLineales;
  
  for ( unsigned int j = 0; j < g.cantVertices( ); ++j )
  {
    dist_s_e = g.getVertice( s_id )->distanciaA( *g.getVertice( j ) );
    distLineales.push_back( dist_s_e );
      
  }
//Reporte de distancias lineales
  std::cout << "\n\n----------Reporte de distancias lineales----------\n";
  std::cout << "Porteria: " << g.getVertice( 0 )->X << "," << g.getVertice( 0 )->Y << std::endl;

  for ( unsigned int j = 1; j < g.cantVertices( ); ++j )
  {
    std::cout << "\nCasa " << j <<"( " <<g.getVertice( j )->X << "," << g.getVertice( j )->Y <<" ): ";
    std::cout << distLineales[ j ] << std::endl;
  }



  // TODO # 6: Encontrar las rutas de costo minimo usando los algoritmos requeridos
  TCaminos caminosPrim = g.prim( 0 );
  TCaminos caminosDijkstra = g.dijkstra( 0 );

  TRuta rutad;
  float cosTotald;
  TRuta rutap;
  float cosTotalp;

  // TODO # 7: Imprimir el informe de Prim
  std::cout<<"\n\n-----------Informe Prim---------------\n";
  for ( unsigned int j = 1; j < g.cantVertices( ); ++j )
  {
    rutap = caminosPrim[j];

      std::cout << "\nCasa "<<j<<" ( "<<g.getVertice(j)->X<<" , "<< g.getVertice(j)->Y<< " ): ";
      if(rutap.size()!=0)
      std::cout << rutap[ 0 ];
      for( unsigned int i = 1; i < rutap.size( ); ++i )
      std::cout << " - "<< rutap[ i ] ;

      std::cout << std::endl;
      std::cout << "Distancia total recorrida: ";
      cosTotalp = 0.0;
      for( unsigned int k = 0; k < rutap.size( ) - 1; ++k )
        cosTotalp += g.obtenerCosto( rutap[ k ], rutap[ k + 1 ] );
      std::cout << cosTotalp << std::endl;

  }




  // TODO # 8: Imprimir el informe de Dijkstra (mismo fomato que informe de Prim)
  std::cout<<"\n\n-----------Informe Dijkstra---------------\n";
  for ( unsigned int j = 1; j < g.cantVertices( ); ++j )
  { 
    rutad = caminosDijkstra[ j ];

    std::cout << "\nCasa "<<j<<" ( "<<g.getVertice(j)->X<<" , "<< g.getVertice(j)->Y<< " ): ";
    if(rutad.size()!=0)
    std::cout << rutad[ 0 ];
    for( unsigned int i = 1; i < rutad.size( ); ++i )
      std::cout << " - "<< rutad[ i ] ;

      std::cout << std::endl;
      std::cout << "Distancia total recorrida: ";
      cosTotald = 0.0;
      for( unsigned int k = 0; k < rutad.size( ) - 1; ++k )
        cosTotald += g.obtenerCosto( rutad[ k ], rutad[ k + 1 ] );
      std::cout << cosTotald << std::endl;

  }



// Imprimir resumen 
std::cout<<"\n\n-----------Informe Completo---------------";
for ( unsigned int j = 1; j < g.cantVertices( ); ++j )
  {
    std::cout << "\n\nCasa "<<j<<" ( "<<g.getVertice(j)->X<<" , "<< g.getVertice(j)->Y<< " ): \n";
    
    
    rutad = caminosDijkstra[ j ];
    rutap = caminosPrim[j];

    std::cout << "Camino segun algoritmo de Dijkstra: ";
    if(rutad.size()!=0)
    std::cout << rutad[ 0 ];
    for( unsigned int i = 1; i < rutad.size( ); ++i )
    std::cout << " - "<< rutad[ i ] ;

    std::cout << std::endl;
  
    std::cout << "Camino segun algoritmo de Prim: ";
    if(rutad.size()!=0)
    std::cout << rutad[ 0 ];
    for( unsigned int i = 1; i < rutap.size( ); ++i )
    std::cout << " - "<< rutap[ i ] ;

    std::cout << std::endl;




    std::cout << std::endl;
    std::cout << "Distancia lineal a porteria: " << distLineales[ j ] << std::endl;


    std::cout << "Distancia total recorrida con algoritmo de Dijkstra: ";
    cosTotald = 0.0;
    for( unsigned int k = 0; k < rutad.size( ) - 1; ++k )
      cosTotald += g.obtenerCosto( rutad[ k ], rutad[ k + 1 ] );
    std::cout << cosTotald << std::endl;

    std::cout << "Distancia total recorrida con algoritmo de Prim: ";
    cosTotalp = 0.0;
    for( unsigned int k = 0; k < rutap.size( ) - 1; ++k )
      cosTotalp += g.obtenerCosto( rutap[ k ], rutap[ k + 1 ] );
    std::cout << cosTotalp << std::endl;

    std::cout << "\nComparacion de Prim con Dijkstra: \n";
    if(cosTotald==cosTotalp){
      std::cout <<"Dijkstra es un camino igual de eficiente que Prim"<<std::endl;
    }else if(cosTotald<cosTotalp){
      std::cout <<"Dijkstra es un camino mas eficiente que Prim con una diferencia de "<< cosTotalp-cosTotald<<std::endl;
    }
    else
    std::cout <<"Prim es un camino mas eficiente que Dijkstra con una diferencia de "<< cosTotald-cosTotalp<<std::endl;

  }

  std::cout << std::endl;
  std::cout << std::endl;


  return( 0 );
}

// eof - taller_4_grafos.cxx
