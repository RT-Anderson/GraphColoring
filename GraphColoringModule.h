// Created by RTAnderson on 4/29/17.

#ifndef GRAPHCOLORING_GRAPHCOLORINGMODULE_H
#define GRAPHCOLORING_GRAPHCOLORINGMODULE_H

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <map>
#include <queue>

class Vertex
{

public:
    int edgeCnt() const {return (int)edgeSet.size();}

    std::unordered_set<int> getEdges(){return edgeSet;}

    Vertex(){};
    Vertex(int inputID)
    {
        id=inputID;
        color=-1;
    }

    void addEdge(int connectedVertex){edgeSet.insert(connectedVertex);}
    void removeEdge(int edge){}
    void addColorRestriction(int colorID)
    {
        while(restrictedColors.size()<=colorID) restrictedColors.push_back(false);
        if (restrictedColors[colorID]==false) colorConstraintCnt++;
        restrictedColors[colorID]=true;

    }

    void reset()
    {
        restrictedColors.clear();
        color=-1;
        colorConstraintCnt=0;
    }

    void assignColor(int maxColor)
    {
        while(restrictedColors.size()<=maxColor) restrictedColors.push_back(false);
        for (int i=0;i<=maxColor;++i)
        {
            if(restrictedColors[i]==false)
            {
                color=i;
                return;
            }
        }
        std::cout << "error: node "<<id<<"couldn't find a valid color\n";

    };

    int id;
    int color;

    int colorConstraintCnt;
    std::unordered_set<int> edgeSet;
    std::vector<bool> restrictedColors;

};

class GraphColoringModule
{

    int bestSolutionColorCnt{2000000000};
    std::vector<int> bestSolution{};
    std::vector<int> bestSolutionIndices{};

    std::map<int,Vertex> nodeMap{};
    std::vector<int> vertexIndices{};


    void importProblemFile(std::string filename)
    {

        bestSolutionColorCnt=2000000000;
        bestSolution.clear();
        bestSolutionIndices.clear();
        nodeMap.clear();
        vertexIndices.clear();

        try
        {
            ulong objectCnt,edgeCnt;
            std::ifstream ss(filename);
            ss >> objectCnt >> edgeCnt;

            if(objectCnt==0) throw std::runtime_error("Input file size states no vectices exist\n");

            for (int i=0;i<objectCnt;++i)
            {
                nodeMap[i]= Vertex(i);
                vertexIndices.push_back(i);
            }

            ulong edgeA, edgeB;
            while(ss>>edgeA>>edgeB)
            {
                nodeMap[edgeA].addEdge((int)edgeB);
                nodeMap[edgeB].addEdge((int)edgeA);
            }

        }

        catch (std::ifstream::failure){std::cerr <<"file: "<<filename<<" could not be opened\n"; }
        catch(const std::runtime_error& e){std::cerr<<e.what();}
        catch(...){std::cerr<<"unspecified error occurred while parsing input file\n";}

    }

    void computeWelshPowellUpperBound()
    {
        //Rearrange nodes by most constrained first
        std::sort(vertexIndices.begin(), vertexIndices.end(),
                  [&](int lhs, int rhs){
                      auto&& lhsEdgeCnt = nodeMap[lhs].edgeCnt();
                      auto&& rhsEdgeCnt = nodeMap[rhs].edgeCnt();
                      return lhsEdgeCnt > rhsEdgeCnt;});

        //Remove graph constraint symmetries
        auto vertexCnt = vertexIndices.size();

        //Convert edges to directed edges that originate from the most constrained vertices first
        for (auto iter=vertexIndices.begin();iter!=vertexIndices.end();++iter)
        {
            for (auto iter2=iter+1;iter2!=vertexIndices.end();++iter2)
            {
                nodeMap[*iter2].removeEdge(*iter);
            }
        }


        //Reset all node values
        for(auto& node:nodeMap) node.second.reset();

        int colorCnt=0;
        for(auto vertexID:vertexIndices)
        {
            //terminate early if a better solution has already been found
            if (colorCnt>=bestSolutionColorCnt) return;

            auto& currVertex = nodeMap[vertexID];

            //local clique cnt exceeds available colors
            if(currVertex.colorConstraintCnt >= colorCnt)
            {
                currVertex.color=colorCnt;

                for(auto neighborID : currVertex.getEdges() )
                {
                    nodeMap[neighborID].addColorRestriction(colorCnt);
                }
                colorCnt++;
            }
            else
            {
                currVertex.assignColor(colorCnt);
                for(auto neighborID : currVertex.getEdges() )
                {
                    nodeMap[neighborID].addColorRestriction(currVertex.color);
                }

            }
        }

        //populate initial best solution
        bestSolutionColorCnt=colorCnt;
        bestSolution.clear();
        for(const auto& node:nodeMap) bestSolution.push_back(node.second.color);
        bestSolutionIndices = vertexIndices;

    };

    void greedySearch(std::vector<int> orderedVertices)
    {

        //Reset all node values
        for(auto& node:nodeMap) node.second.reset();

        int colorCnt=0;
        for(auto vertexID:orderedVertices)
        {
            //terminate early if a better solution has already been found
            if (colorCnt>=bestSolutionColorCnt) return;

            auto& currVertex = nodeMap[vertexID];

            //local clique cnt exceeds available colors
            if(currVertex.colorConstraintCnt >= colorCnt)
            {
                currVertex.color=colorCnt;

                for(auto neighborID : currVertex.getEdges() )
                {
                    nodeMap[neighborID].addColorRestriction(colorCnt);
                }
                colorCnt++;
            }
            else
            {
                currVertex.assignColor(colorCnt);
                for(auto neighborID : currVertex.getEdges() )
                {
                    nodeMap[neighborID].addColorRestriction(currVertex.color);
                }

            }
        }

        //populate initial best solution
        if (colorCnt<bestSolutionColorCnt) {
            bestSolutionColorCnt = colorCnt;
            std::cout << colorCnt << std::endl;
            bestSolution.clear();
            for (const auto &node:nodeMap) bestSolution.push_back(node.second.color);
            bestSolutionIndices=orderedVertices;
        }

    };

    void printSolution()
    {
        std::cout << bestSolutionColorCnt<<" 0\n";
        for (const auto& color:bestSolution) std::cout << color <<" ";

        std::cout <<"\n";
    }

/*
    std::vector<std::vector<int>> generateVariations(std::vector<int> partialSolution, std::unordered_set<int> availableNodes)
    {
        auto inputs = std::make_pair(partialSolution, availableNodes);
        std::queue<std::pair<std::vector<int>, std::unordered_set<int>>> partialSolutions{};
        partialSolutions.push(inputs);

        while (!partialSolutions.front().second.empty()){
            for (const auto v : partialSolutions.front().second) {
                auto a = partialSolutions.front().first;
                a.push_back(v);
                auto b = partialSolutions.front().second;
                b.erase(v);
                auto tmpSol = std::make_pair(a, b);
                partialSolutions.push(tmpSol);
            }
            partialSolutions.pop();
        }

        std::vector<std::vector<int>> output;
        while(!partialSolutions.empty())
        {
            output.push_back(partialSolutions.front().first);
            partialSolutions.pop();
        }

        return output;

    };
    */

    //Makes random swaps to best solution; keeps new solution if improvement
    void randomSearchGA(int attemptCnt,int swapCnt)
    {
        for (int i=0;i<attemptCnt;++i)
        {
            auto tmpIndices = bestSolutionIndices;
            auto maxSwap=std::rand()%swapCnt+1;
            for (int j=0;j<maxSwap;++j) {
                auto offsetA = std::rand() % tmpIndices.size();
                auto offsetB = std::rand() % tmpIndices.size();
                std::iter_swap(tmpIndices.begin() + offsetA, tmpIndices.begin() + offsetB);
            }
            greedySearch(tmpIndices);
            if (i % 250==0) std::cout<<"run "<<i<<"\n";
        }
    }

    //Modifies Welsh-Powell algorithm by randomly shuffling nodes with the same cover value
    void randomSearchOrderedByCover(int attemptCnt) {
        std::map<int, std::vector<int>, std::greater<int>> coverMap{};
        for (const auto& vertex:nodeMap)
            coverMap[vertex.second.edgeCnt()].push_back(vertex.first);

        for (int attempt=0;attempt<attemptCnt;++attempt) {
            for (auto i=0;i<coverMap.size();++i)
            //for (auto &coverVector:coverMap)
                if (coverMap[i].size() > 1)
                    std::random_shuffle(coverMap[i].begin(), coverMap[i].end());
            std::vector<int> solution{};
            solution.reserve(nodeMap.size());
            for (const auto &coverVector:coverMap)
                solution.insert(solution.end(), coverVector.second.begin(), coverVector.second.end());
            greedySearch(solution);
        }
    }

public:
    void solve(std::string filename)
    {
        int attemptCnt=250;
        int swapCnt = 20;
        
        importProblemFile(filename);
        computeWelshPowellUpperBound();
        //randomSearchGA(attemptCnt,swapCnt); //disabled random-search heuristic; OrderedByCover heuristic provides better results
        randomSearchOrderedByCover(attemptCnt);

        printSolution();
    }
};


#endif //GRAPHCOLORING_GRAPHCOLORINGMODULE_H
