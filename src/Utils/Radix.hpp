/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */


#ifndef Antipatrea__Radix_HPP_
#define Antipatrea__Radix_HPP_

#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cfloat>
#include <climits>
#include <limits>
#include <cmath>
#include <cassert>
#include <string>
#include <algorithm>
#include "Utils/Misc.hpp"

namespace Antipatrea {

    class Link {
    public:
        int succid;
        long weight;
        Link *next;

        Link(int n, long w, Link *p) {
            next = p;
            succid = n;
            weight = w;
        }
    };

    class Node {
    public:
        enum Label {
            unlabelled, labelled, scanned
        };
        int m_id;

        Label state;
        Link *edges;
        long element;
        int bucket;
        Node *pred;
        Node *succ;
        long x;
        long y;

        Node *m_parent;
        long m_parentCost;

        Node(int id, long xcoord, long ycoord) {
            m_id = id;
            m_parent = NULL;
            m_parentCost = 0;

            x = xcoord;
            y = ycoord;
            state = unlabelled;
            pred = succ = 0;
            element = 0;
            edges = 0;
        }

        Node(long v) {
            state = unlabelled;
            pred = succ = 0;
            element = v;
            edges = 0;
        }
    };

    class Radix {
    public:
        long S;
        int B;
        long n; // number of nodes
        Node **buckets;
        long *u;
        long *b;

        Radix() {
            S = std::numeric_limits<long>::max();
            B = (int) (ceil(log(S) / log(2))) + 2;
            buckets = new Node *[B];
            b = new long[B];
            u = new long[B];
            for (int i = 0; i < B; i++) {
                buckets[i] = 0;
                b[i] = u[i] = 0;
            }
            b[0] = 1;
            b[B - 1] = std::numeric_limits<long>::max();
            for (int i = 1; i < B - 1; i++)
                b[i] = 1L << (i - 1);
            u[B - 1] = std::numeric_limits<long>::max();
            n = 0;
        }

        virtual ~Radix() {
            delete[] b;
            delete[] u;
            delete[] buckets;

        }

        long size() {
            return n;
        }

        bool empty() {
            return (n == 0);
        }

        Node *next(Node *p) {
            if (p->succ != 0)
                return p->succ;
            else {
                int next = p->bucket + 1;
                while ((next < B) && (buckets[next] == 0))
                    next++;
                if (next == B)
                    return 0;
                else
                    return buckets[next];
            }
        }

        void insert_node(Node *p, int i) {
            p->succ = buckets[i];
            if (buckets[i] != 0)
                buckets[i]->pred = p;
            p->pred = 0;
            p->bucket = i;
            buckets[i] = p;
        }

        void extract_node(Node *p) {
            if (p->pred != 0) {
                Node *q = p->pred;
                q->succ = p->succ;
            } else {
                buckets[p->bucket] = p->succ;
            }
            if (p->succ != 0) {
                Node *q = p->succ;
                q->pred = p->pred;
            }
        }

        void adjust(long m, int t) {
            int i;
            u[0] = m;
            for (i = 1; i < t; i++) {
                u[i] = u[i - 1] + b[i];
                if (u[i] > u[t])
                    break;
            }
            for (; i < t; i++)
                u[i] = u[t];
        }

        int find(Node *p, int i) {
            if (p->element == u[0])
                return 0;
            while (p->element <= u[--i]);
            return i + 1;
        }

        Node *top() {
            return buckets[0];
        }

        Node *insert(Node *p) {
            long k = p->element;
            if (n > 0) {
                //  if (k < u[0]) cout  << "Error insert" << k << " < " << u[0] << endl;
                int i = find(p, B - 1);
                insert_node(p, i);
            } else {
                adjust(k, B - 1);
                buckets[0] = p;
                p->bucket = 0;
            }
            n++;
            return p;
        }

        void decrease(Node *x, long k) {
            //	if (k >= x->element) cout  << "Error decrease" << k << " >= " << x->element << endl;
            //	if (k < u[0]) cout  << "Error decrease" << k << " < " << u[0] << endl;
            x->element = k;
            if (k <= u[x->bucket - 1]) {
                extract_node(x);
                int i = find(x, x->bucket);
                insert_node(x, i);
            }
        }

        Node *extract() {
            for (int i = 0; i < B; i++) {
                Node *p = buckets[i];
                if (p != 0) {
                    extract_node(p);
                    n--;
                    return p;
                }
            }
            return 0;
        }

        Node *extract(Node *x) {
            int i = x->bucket;
            extract_node(x);
            if ((n > 1) && (i == 0) && (buckets[0] == 0)) {
                int j = 1;
                while (buckets[j] == 0)
                    j++;
                Node *p = buckets[j];
                Node *d = p->succ;
                while (d != 0) {
                    if (d->element < p->element)
                        p = d;
                    d = d->succ;
                }
                adjust(p->element, j);
                extract_node(p);
                insert_node(p, 0);
                p = buckets[j];
                while (p != 0) {
                    Node *q = p->succ;
                    extract_node(p);
                    int l = find(p, j);
                    insert_node(p, l);
                    p = q;
                }
            }
            n--;
            return x;
        }
    };

    class RadixGraph {
    public:
        std::vector<Node *> nodes;
        std::vector<Link *> m_links;


        RadixGraph(int n) {
            for (int i = 0; i < n; i++)
                nodes.push_back(new Node(i, rand() % 10000, rand() % 10000));
        }

        virtual ~RadixGraph(void) {
            DeleteItems<Node *>(nodes);
            DeleteItems<Link *>(m_links);

        }

        int findId(const int id) {
            if (id < nodes.size() && nodes[id]->m_id == id)
                return id;
            for (int i = nodes.size() - 1; i >= 0; --i)
                if (nodes[i]->m_id == id)
                    return i;
            return -1;

        }


        void addEdge(int i, int j, long w) {
            auto link = new Link(j, w, nodes[i]->edges);

            nodes[i]->edges = link;
            m_links.push_back(link);
        }

        void init() {
            for (int i = 0; i < nodes.size(); i++) {
                nodes[i]->pred = nodes[i]->succ = 0;
            }
        }

        Node *specificNode(int i) {
            return nodes[i];
        }

        Node *randomNode() {
            return nodes[rand() % nodes.size()];
        }

        long path(Node *s, std::vector<int> &ids) {
            long cost = 0;

            ids.clear();

            while (s != NULL) {
                ids.push_back(s->m_id);
                cost += s->m_parentCost;
                s = s->m_parent;
            }
            //ReverseItems<int>(ids);

            return cost;

        }

        void dijkstra(Node *s) {
            std::vector<int> vids;

            int n = nodes.size();
            int v = 0;
            for (int i = 0; i < n; i++) {
                nodes[i]->state = Node::unlabelled;
                nodes[i]->pred = nodes[i]->succ = 0;
                nodes[i]->m_parent = NULL;
                nodes[i]->m_parentCost = 0.0;

            }

            // cout  << "Nodes initialized" << endl;
            s->element = 0;
            s->state = Node::labelled;
            Radix *heap = new Radix();
            heap->insert(s);
            //cout  << "Heap initialized" << endl;
            while (heap->size() != 0) {
                v++;
                Node *t = heap->top();
                t->state = Node::scanned;
                long d = t->element;
                //cout << "shortest path to " << t->m_id << " with cost " << t->element << " and cost " << path(t, vids) << std::endl;
                //	    cout  << "Deleting minimum " << d << endl;
                for (Link *l = t->edges; l != 0; l = l->next) {
                    Node *u = nodes[l->succid];
                    //		cout  << "Next: " << u << endl;
                    long c = d + l->weight;
                    if (u->state == Node::scanned);
                    else if (u->state == Node::unlabelled) {
                        u->element = c;
                        //	        cout  << "Inserting with cost " << c << endl;
                        heap->insert(u);
                        u->state = Node::labelled;

                        u->m_parent = t;
                        u->m_parentCost = l->weight;

                    } else if (u->element > c) {
                        //		  cout  << "Decreasing to cost " << c << endl;
                        heap->decrease(u, c);

                        u->m_parent = t;
                        u->m_parentCost = l->weight;

                    }
                }
                heap->extract(t);
            }
            //cout  << "Expanded nodes " << v << endl;
            delete heap;

        }
    };

}

#endif
