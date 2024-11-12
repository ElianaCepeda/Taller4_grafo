#ifndef GRAFO_HXX
#define GRAFO_HXX
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <limits.h>

template <class T>
class Grafo {
    private:
        std::vector<T> vertices;
        double** aristas;
    public:
        Grafo();
        void setVertices(std::vector<T> vertices);
        void setAristas (double** aristas);
        const std::vector<T>& getVertices() const;
        double** getAristas();
        T *getVertice(int indice);
        int cantVertices();
        int cantAristas();
        int buscarVertice(T ver);
        bool insertarVertice(T ver);
        bool insertarArista(int ori, int des, float cos);
        bool insAristaNoDir(int ori, int des, float cos);
        int buscarArista(T origen, T destino);
        bool eliminarVertice(T ver);
        bool eliminarArista (T origen, T destino);
        bool elimAristaNoDir(T ori, T des);
        std::vector<T> vecinosVertice(T ver);
        std::vector<unsigned long > indicesVecinos(unsigned long id);
        void plano();
        std::vector<T> DFS(T ver_inicial);
        std::vector<T> BFS(T ver_inicial);   
        std::vector< std::vector<unsigned long> > dijkstra(unsigned long  i_fuente);
        void encontrarPuentes();
        bool esEuler(); 
        std::vector<int> circuitoEuler();
        void floydWarshall();
        void kruskal(); 
        std::vector< std::vector<unsigned long> > prim(int inicio);
        double obtenerCosto(int id1, int id2);

};

#endif