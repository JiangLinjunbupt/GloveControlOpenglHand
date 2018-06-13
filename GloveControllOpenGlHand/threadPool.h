#pragma once
#include<thread>
#include<mutex>
#include<functional>  //C++ 11�����˺��������׼��<functional>
#include<string>
#include<condition_variable>  //��������
#include<deque>
#include<vector>
#include<memory>            // For std::shared_ptr<>
using namespace std;

class nocopyable//��nocopyable���ɿ������̳и��������Ҳ���ɿ���
{
private:
	nocopyable(const nocopyable& x) = delete; //C++11ʹ��delete�ؼ�����ʽָʾ�����������ɺ�����Ĭ�ϰ汾,ͬʱҲ��ֹ���ظú���
	nocopyable& operator=(const nocopyable& x) = delete;
public:
	nocopyable() = default;//default��ʽ��ָʾ���������ɸú�����Ĭ�ϰ汾
	~nocopyable() = default;
};

class ThreadPool :public nocopyable
{
public:
	typedef std::function<void()> Task;//��������ͨ������lambda���ʽ��bind���ʽ����������ֵ
	explicit ThreadPool();
	~ThreadPool();

	void start(int numThraeds);//�����߳���������numThreads���߳�
	void stop();//�̳߳ؽ���
	void run(const Task& f); //�������f���������
	void setMaxQueueSize(int maxSize) { _maxQueueSize = maxSize; }
private:
	bool isFull();//��������Ƿ�����
	void runInThread(); //�̵߳���ں��������ڲ�����take()ȡ����
	Task take();//�����������ȡ��������

	mutex _mutex;
	condition_variable _notEmpty;
	condition_variable _notFull;
	vector<thread> _threads;
	deque<Task> _queue;  //˫�˶���deque,�ܸ�Ч����ɾ��������ͷ��Ԫ��
	int _maxQueueSize; //������пɴ�����������
	bool _running;

	condition_variable _allTaskDone;
	int _submitTaskNum = -1;  //�ύ���̳߳ص�������
	int _fulfillTaskNum = 0;//�̳߳�����ɵ���������
public:
	void ensureTaskCompleted(int submitTaskNum);
	int get_maxQueueSize();
};

