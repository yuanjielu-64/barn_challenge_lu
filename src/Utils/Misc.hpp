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

#ifndef Antipatrea__Misc_HPP_
#define Antipatrea__Misc_HPP_

#include "Utils/PseudoRandom.hpp"
#include <cctype>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <ostream>
#include <vector>
#include <cmath>

namespace Antipatrea {
    static inline void ToLower(char s[]) {
        for (int i = 0; s[i] != '\0'; ++i)
            s[i] = std::tolower(s[i]);
    }

    static inline void ToUpper(char s[]) {
        for (int i = 0; s[i] != '\0'; ++i)
            s[i] = std::toupper(s[i]);
    }

    static inline double StrToDouble(const char s[]) { return strtod(s, NULL); }

    static inline long int StrToInt(const char s[]) { return strtol(s, NULL, 0); }

    static inline bool StrToBool(const char s[]) { return strcmp(s, "false") != 0; }

    static inline bool StrSameContent(const char s1[], const char s2[]) {
        if (s1 == NULL)
            return s2 == NULL;
        if (s2 == NULL)
            return s1 == NULL;

        int i = 0;
        for (; s1[i] != '\0' && s2[i] != '\0'; ++i)
            if (std::tolower(s1[i]) != std::tolower(s2[i]))
                return false;
        return s1[i] == s2[i];
    }

    template<typename Item>
    static inline Item *AllocArray(const int n) {
        return (Item *) calloc(n, sizeof(Item));
    }

    template<typename Item>
    static inline void FreeArray(Item a[]) {
        if (a)
            free(a);
    }

    template<typename Item>
    static inline void CopyArray(Item dest[], const int n, const Item src[]) {
        memcpy(dest, src, n * sizeof(Item));
    }

    template<typename Item>
    static inline void DeleteItems(const int n, Item items[]) {
        for (int i = 0; i < n; ++i)
            if (items[i])
                delete items[i];
    }

    template<typename Item>
    static inline void DeleteItems(std::vector<Item> &items) {
        if (items.size() > 0)
            DeleteItems<Item>(items.size(), &(items[0]));
    }

    template<typename Item>
    static inline int FindItem(const int n, const Item items[], const Item &item) {
        for (int i = 0; i < n; ++i)
            if (items[i] == item)
                return i;
        return -1;
    }

    template<typename Item>
    static inline int FindItem(const std::vector<Item> &items, const Item &item) {
        if (items.size() == 0)
            return -1;
        return FindItem<Item>(items.size(), &(items[0]), item);
    }

    template<typename Item>
    static inline void ReverseItems(const int n, Item items[]) {
        Item tmp;
        for (int i = 0, j = n - 1; i < j; ++i, --j) {
            tmp = items[i];
            items[i] = items[j];
            items[j] = tmp;
        }
    }

    template<typename Item>
    static inline void ReverseItems(std::vector<Item> &items) {
        if (items.size() > 0)
            ReverseItems<Item>(items.size(), &(items[0]));
    }

/**
 *@brief Generate a random subset of <tt>r</tt> out of
 *       <tt>n</tt> elements.
 *@param n number of elements in the array <tt>items</tt>
 *@param items array of elements
 *@param r number of elements in the subset
 *
 *@par Description:
 *  A subset of <tt>r</tt> elements out of <tt>n</tt> elements
 *  can be selected uniformly at random by selecting the
 *  <tt>i</tt>-th element, <tt>i = 0, 1, ..., r - 1</tt>, of
 *  the <tt>r</tt>-subset of <tt>p</tt> as follows:
 *  - generate uniformly at random an index <tt>j</tt> in the
 *    integer interval <tt>[i, n - 1]</tt>
 *  - swap elements <tt>p[i]</tt> and <tt>p[j]</tt>
 *  .
 *  At the end of this procedure, the elements <tt>p[0], p[1],
 *  ..., p[r - 1]</tt> correspond to an <tt>ordered</tt>
 *  <tt>r</tt>-subset selected uniformly at random from the
 *  original <tt>n</tt>-set <tt>p</tt>. The elements <tt>p[r],
 *  ..., p[n - 1]</tt> contain the original elements of
 *  <tt>p</tt> that were not selected.
 *  \n\n
 *  The number of ways of choosing an <tt>ordered</tt>
 *  <tt>r</tt>-subset out of an <tt>n</tt>-set <tt>p</tt> is
 *  given by
 *  <CENTER><tt>P(r, n) = n! / (n - r)!</tt>.</CENTER>
 *  For example, for <tt>r = 2</tt> there are <tt>4! / (4 -
 *  2)! = 12</tt> <tt>ordered</tt> <tt>r</tt>-subsets of <tt>p
 *  = {0, 1, 2, 3}</tt>, namely, <CENTER><tt>{0, 1}, {0, 2},
 *  {0, 3}, {1, 0}, {1, 2}, {1, 3}, {2, 0}, {2, 1}, {2, 3}, {3,
 *  0}, {3, 1}, {3, 2}</tt>.</CENTER> The <tt>ordered</tt>
 *  <tt>r</tt>-subset is said to be selected uniformly at
 *  random  when it is selected with probability <CENTER><tt>1
 *  / P(r, n)</tt></CENTER> out of all <tt>P(r, n)</tt>
 *  possible <tt>ordered</tt> <tt>r</tt>-subsets of the
 *  <tt>n</tt>-set <tt>p</tt>. Similarly,  the number of ways
 *  of choosing an \em unordered <tt>r</tt>-subset out of an
 *  <tt>n</tt>-set <tt>p</tt> is given by <CENTER><tt>C(r, n)
 *  = P(r, n) / r! = n! / ((n - r)! r!)</tt>.</CENTER>
 *  For example, for <tt>r = 2</tt> there are <tt>4! / ((4 -
 *  2)! 2!) = 6</tt>\em unordered <tt>r</tt>-subsets of <tt>p
 *  = {0, 1, 2, 3}</tt>, namely, <CENTER><tt>{0, 1}, {0, 2},
 *  {0, 3}, {1, 2}, {1, 3}, {2, 3}</tt>.</CENTER>
 *  The \em unordered <tt>r</tt>-subset is said to be selected
 *  uniformly at random when it is selected with probability
 *  <CENTER><tt>1 / C(r, n)</tt></CENTER>
 *  out of all <tt>C(r, n)</tt> possible \em unordered
 *  <tt>r</tt>-subsets of the <tt>n</tt>-set <tt>p</tt>.
 *  \n\n
 *  Observe that each \em unordered <tt>r</tt>-subset
 *  corresponds to <tt>r!</tt> <tt>ordered</tt>
 *  <tt>r</tt>-subsets obtained by generating all possible
 *  permutations of the elements of the \em unordered
 *  <tt>r</tt>-subset.  Hence, the same procedure that selects
 *  uniformly at random an <tt>ordered</tt> <tt>r</tt>-subset
 *  of the <tt>n</tt>-set <tt>p</tt>, can be used to select
 *  uniformly at random an <tt>unordered</tt>
 *  <tt>r</tt>-subset of the <tt>n</tt>-set <tt>p</tt>. The
 *  \em unordered <tt>r</tt>-subset of <tt>p</tt> is selected
 *  when one of the <tt>r!</tt> corresponding <tt>ordered</tt>
 *  <tt>r</tt>-subsets is selected. Since an <tt>ordered</tt>
 *  <tt>r</tt>-subset is selected with probability <tt>1 / P(r,
 *  n)</tt>, then an \em unordered <tt>r</tt>-subset is
 *  selected with probability  <CENTER><tt> r! (1/ P(r, n)) =
 *  1 / C(r, n)</tt>. </CENTER>
 *
 *@remarks
 *  The values of <tt>n</tt> and <tt>r</tt> should satisfy the
 *  following constraint:
 *  - <tt>0 <= r <= n</tt>
 *
 */
    template<typename Item>
    static void PermuteItems(const int n, Item items[], const int r) {
        for (int i = 0; i < r; i++) {
            int j = RandomUniformInteger(i, n - 1);
            Item t = items[i];
            items[i] = items[j];
            items[j] = t;
        }
    }

/**
 *@brief Generate a random subset of <tt>r</tt> elements out of
 *       the elements in the vector <tt>items</tt>.
 *@remarks
 * - It calls <tt>PermuteItems<Item>(n, p, r)</tt> with <tt>p</tt>
 *   as the memory address of the first element in the vector <tt>items</tt> and
 *   <tt>n</tt> as the number of elements in the vector <tt>items</tt>.
 */
    template<typename Item>
    static inline void PermuteItems(std::vector<Item> &items, const int r) {
        if (items.size() > 0)
            PermuteItems<Item>(items.size(), &(items[0]), r);
    }

/**
 *@brief Generate the next permutation in the lexicographic
 *       order.
 *@param n number of elements
 *@param items array of elements
 *
 *@returns
 *  <tt>true</tt> iff <tt>items</tt> is already the largest
 *  permutation in the lexicographic order
 *
 *@par Description:
 *  A permutation A is said to be larger than another
 *  permutation B iff there exists an integer <tt>j</tt> such
 *  that <CENTER><tt>A[0..j-1] = B[0..j-1]</tt> and <tt>A[j] >
 *  B[j]</tt>,</CENTER> i.e., it is the same as comparing
 *  numbers where the digits of the number form the
 *  permutation.
 *  \n\n
 *  The next permutation of <tt>A</tt> is defined as the
 *  smallest possible permutation <tt>B</tt> larger than
 *  <tt>A</tt>.
 *  \n\n
 *  The procedure to generate the next permutation in the
 *  lexicographic order is as follows:
 *  - Starting from right to left examine the array to find
 *    the largest tail that is in descending
 *    order. Permutation looks like <CENTER><tt>p[0] < ... <
 *    p[i-2] < p[i-1] < p[i] > p[i+1] > p[i+2] >... >
 *    p[n-1].</tt></CENTER>
 *  - Since we are after the next largest permutation we
 *    should keep <tt>p[0], p[1],...,p[i-2]</tt> unchanged and
 *    change <tt>p[i-1]</tt> to something larger but as small
 *    as possible. Thus, we need to find <tt>p[k] >
 *    p[i-1]</tt> among the longest tail which is the smallest
 *    possible.
 *  - Swap <tt>p[k]</tt> with <tt>p[i-1]</tt>.
 *  - Reverse tail <tt>p[i..n-1]</tt>.
 *
 *@remarks
 *  <tt>Type</tt> must support the less-than comparison
 *  operator <tt> < </tt>.
 */
    template<typename Type>
    static inline bool NextPermutation(const int n, Type items[]) {
        int i, j, k, t;

        // starting from right to left examine the array to find
        // the largest tail that is in descending order
        for (i = n - 1; i > 0 && items[i] < items[i - 1]; i--);

        // the longest tail is the entire permutation
        // i.e.,  n, n-1,..., 2, 1 is the largest possible permutation
        if (i <= 0)
            return false;

        // permutation looks like
        // items[0] < ... < items[i-2] < items[i-1] < items[i] > items[i+1] >
        // items[i+2] >... > items[n-1]

        // since we are after next largest permutation we should keep
        // items[0], items[1],...,items[i-2] unchanged and change items[i-1] to
        // something  larger but as small as possible.  Need to find items[k] >
        // items[i-1] among the longest tail which is the  smallest possible
        for (k = i, j = i + 1; j < n; j++)
            if (items[j] > items[i - 1] && items[j] < items[k])
                k = j;

        // swap items[k] with items[i-1]
        t = items[k];
        items[k] = items[i - 1];
        items[i - 1] = t;

        // reverse tail items[i..n-1]
        k = (n - i) / 2;
        for (j = 0; j < k; j++) {
            t = items[i + j];
            items[i + j] = items[n - 1 - j];
            items[n - 1 - j] = t;
        }
        return true;
    }

    static inline bool NextCombination(const int n, int combo[], const int r) {
        if (r == 0 || combo[0] >= n - r)
            return false;

        int i = r - 1;
        while (combo[i] == n - r + i)
            i--;

        combo[i] = combo[i] + 1;
        for (int j = i + 1; j < r; j++)
            combo[j] = combo[i] + j - i;
        return true;
    }

    static inline bool NextGroupCombination(const int n, int vals[],
                                            const int max[]) {
        int pos = n - 1;
        while (pos >= 0 && vals[pos] == 0)
            --pos;
        if (pos < 0)
            return false;

        --vals[pos];
        if (vals[pos] >= 0) {
            for (int i = pos + 1; i < n; ++i)
                vals[i] = max[i];
        }

        return true;
    }

    static inline void OutputFormat(std::ostream &out) {
        auto ff = out.flags();
        ff &= ~std::ostream::scientific;
        ff |= out.fixed;
        out.flags(ff);
        out.precision(12);
    }

    template<typename Item>
    static inline void AssignItems(Item a[], const int n, ...) {
        va_list args;
        va_start(args, n);
        for (int i = 0; i < n; ++i)
            a[i] = va_arg(args, Item);
        va_end(args);
    }

    template<typename Item>
    static inline void PrintItems(std::ostream &out, const Item a[], const int n) {
        for (int i = 0; i < n; ++i)
            out << a[i] << " ";
        out << std::endl;
    }

    template<typename Item>
    static inline void PrintItems(std::ostream &out, std::vector<Item> &items) {
        PrintItems(out, &items[0], items.size());
    }

    static inline
    double AdjustTimeStep(const double dt) {
        return 1.0 / ceil(1.0 / dt);

    }

    static inline
    double CleanValue(const double d) {
        return std::round(d * 10) / 10;
    }


}

#endif
