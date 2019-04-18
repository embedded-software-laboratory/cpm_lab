#pragma once
#include "default.h"

template<class T>


class DetectedObjectsBuffer
{

private:
	/**
		id which is expected to be added next to the queue
	*/
	int next_expected_id = 0;

	/**
		queue where the objects are saved
	*/
	std::queue<T> queue;

	/**
		map which stores all elements which are already added
		although one id is missing
	*/
	std::map<int, T> additional;

	/**
		lock for the queue
	*/
	std::mutex lock;

	/**
		number of objects that were stored in the queue - overall
		-> only for evaluational purpose
	*/
	int num = 0;

public:
	DetectedObjectsBuffer<T>() {}
	~DetectedObjectsBuffer() {}

	/**
		enqeue an objects or save it in the map for later
	*/
	bool enqueue(const int id, const T &element) {

		assert(id >= next_expected_id);
		assert(additional.find(id) == additional.end());

		lock.lock();

		if (id > next_expected_id) {
			//save for later insertion
			additional[id] = element;
			lock.unlock();
			return true;
		}

		//id == next_expected_id

		queue.push(element);
		next_expected_id = id + 1;
		num++;

		typename std::map<int,T>::iterator it = additional.find(next_expected_id);
		while (it != additional.end()) {
			queue.push(it->second);
			num++;
			additional.erase(it);

			next_expected_id++;
			it = additional.find(next_expected_id);
		}

		lock.unlock();
		return true;

	}

	/**
		dequeue an element 
		return true if is successful
		if the queue is empty return false
	*/
	bool dequeue(T &element) {

		lock.lock();

		if (queue.empty()) {
			lock.unlock();
			return false;
		}


		element = queue.front();
		queue.pop();
		lock.unlock();
		return true;
	}

	//for evaluation

	/**
		get number of objects that were stored in this queue
	*/
	int getNum() {
		return num;
	}

};

