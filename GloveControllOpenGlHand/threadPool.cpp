#include "ThreadPool.h"
#include<iostream>
using namespace std;
ThreadPool::ThreadPool() :_maxQueueSize(0), _running(false) { }
ThreadPool::~ThreadPool()
{
	if (_running)
	{
		stop();
	}
}
void ThreadPool::start(int numThreads)
{
	if (!_threads.empty())
	{
		cout << "vector<thread> _threads��Ϊ��" << endl;
		exit(EXIT_FAILURE);
	}
	_running = true;
	_threads.reserve(numThreads); //reserver����������vectorԤ����洢����С����capacity��ֵ
	for (int i = 0; i < numThreads; i++)
	{
		_threads.push_back(thread(&ThreadPool::runInThread, this));//��this->runInThread()��Ϊ�̺߳���
	}
}
void ThreadPool::stop()
{
	{
		unique_lock<mutex> lock(_mutex); //��ȡ������_mutex������Ȩ��������_mutex.lock()�������
		_running = false;
		_notEmpty.notify_all();
	}                                 //ִ������������ݣ�lock���������������������_mutex.unlock()����
	for (size_t i = 0; i < _threads.size(); i++)
	{
		_threads[i].join();
	}
}
void ThreadPool::run(const Task& f)
{
	if (_threads.empty()) { f(); }
	else
	{
		unique_lock<mutex> lock(_mutex);
		while (isFull())
		{
			_notFull.wait(lock);
		}
		if (isFull())
		{
			cout << "!isFull()" << endl;
			exit(EXIT_FAILURE);
		}
		_queue.push_back(f);
		_notEmpty.notify_one();
		//cout << "��������̳߳�" << endl;
	}
}
ThreadPool::Task ThreadPool::take()
{
	unique_lock<mutex> lock(_mutex);
	while (_queue.empty() && _running)  //stop()��_running��Ϊfalse
	{
		_notEmpty.wait(lock);  //��������������������ĳ���̵߳��� notify_* ���Ѻ�����������������ٴ��ж�ѭ������
	}
	Task task;
	if (!_queue.empty())
	{
		task = _queue.front();  //������Ԫ�ص�����
		_queue.pop_front();//ɾ��˫�˶�������ǰһ��Ԫ��
		if (_maxQueueSize > 0)
		{
			_notFull.notify_one();
		}
		//cout << "���̳߳�ȡ����" << endl;
	}
	return task;
}
bool ThreadPool::isFull()
{
	return _maxQueueSize > 0 && (int)_queue.size() >= _maxQueueSize;
}
void ThreadPool::runInThread()
{
	try
	{
		while (_running)
		{
			Task task = take();
			if (task)
			{
				task();
				_mutex.lock();
				_fulfillTaskNum++;
				if (_fulfillTaskNum == _submitTaskNum)
				{
					_allTaskDone.notify_one();
				}
				_mutex.unlock();
			}
		}
	}
	catch (const exception& ex)
	{
		fprintf(stderr, "reason: %s\n", ex.what());
		abort();
	}
	catch (...)
	{
		fprintf(stderr, "exception caught in ThreadPool \n");
	}
}


int ThreadPool::get_maxQueueSize()
{
	return _maxQueueSize;
}

void ThreadPool::ensureTaskCompleted(int submitTaskNum)
{
	unique_lock<mutex> lck(_mutex);
	_submitTaskNum = submitTaskNum;
	_allTaskDone.wait(lck, [this, &submitTaskNum] {return this->_fulfillTaskNum == submitTaskNum; });
	_submitTaskNum = -1;//��Ϊ-1�������´ε��øú���ǰ��runInThread()�в��ỽ��_allTaskDone
	_fulfillTaskNum = 0;
}