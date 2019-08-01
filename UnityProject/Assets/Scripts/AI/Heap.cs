using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;

public class Heap<T> where T : IHeapItem<T>
{

    T[] items;
    int currentCount;

    //heap, used for optimization of A* search, improves speed of searching through an openset for optimal node
    public Heap(int _maxHSize)
    {
        items = new T[_maxHSize];

    }

    public void Add(T item) //add item to the heap
    {
        item.HIndex = currentCount;
        items[currentCount] = item;
        SortUp(item);
        currentCount++;
    }

    public T RemoveFirst()  //remove first item from the heap
    {
        T firstItem = items[0];
        currentCount--;
        items[0] = items[currentCount];
        items[0].HIndex = 0;
        SortDown(items[0]);
        return firstItem;
    }

    public void UpdateItem(T item)
    {
        SortUp(item);
    }

    public int Count
    {
        get
        {
            return currentCount;
        }
    }

    public bool Contains(T item)
    {
        return Equals(items[item.HIndex], item);
    }

    void SortDown(T item)
    {
        while(true)
        {
            int childIndexLeft = item.HIndex * 2 + 1;   //find child item to the left
            int childIndexRight = item.HIndex * 2 + 2;  //find child item to the right
            int swapIndex = 0;

            if (childIndexLeft < currentCount)  //if there is a child on the left
            {
                swapIndex = childIndexLeft; //set swap index to left child
                if (childIndexRight < currentCount) //if there is a child on the right
                    if (items[childIndexLeft].CompareTo(items[childIndexRight]) < 0)    //if child to the right has lower F cost than one on the left 
                        swapIndex = childIndexRight;    //set swap index to right child

                if (item.CompareTo(items[swapIndex]) < 0)   //check whether child has lower F score than its parent
                    Swap(item, items[swapIndex]);   //swap item with its child
                else
                    return;
            }
            else
                return;
        }
    }

    void SortUp(T item)
    {
        int parentIndex = (item.HIndex - 1) / 2;    //get parent heap index

        while(true) //while parent has higher F score than child, swap
        {
            T parentItem = items[parentIndex];
            if (item.CompareTo(parentItem) > 0)
            {
                Swap(item, parentItem);
            }
            else
                break;

            parentIndex = (item.HIndex - 1) / 2;
        }
    }

    void Swap( T _itemA, T _itemB)  //swap heap items
    {
        items[_itemA.HIndex] = _itemB;
        items[_itemB.HIndex] = _itemA;
        int aIndex = _itemA.HIndex;
        _itemA.HIndex = _itemB.HIndex;
        _itemB.HIndex = aIndex;
    }

}

public interface IHeapItem<T> : IComparable<T>
{
    int HIndex
    {
        get;
        set;
    }
}