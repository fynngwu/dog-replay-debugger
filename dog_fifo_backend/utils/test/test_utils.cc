#include <atomic>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "utils/async_message_process.h"
#include "utils/timer.h"

// ============================================================
//  Timer tests
// ============================================================

void test_timer_basic() {
    std::cout << "=== test_timer_basic ===" << std::endl;
    Timer::Clear();

    Timer::Evaluate([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }, "sleep_100ms", true);

    double avg = Timer::GetMeanTime("sleep_100ms");
    std::cout << "  mean: " << avg << " ms (expect ~100)" << std::endl;

    Timer::PrintAll();
}

void test_timer_multiple_calls() {
    std::cout << "=== test_timer_multiple_calls ===" << std::endl;
    Timer::Clear();

    for (int i = 0; i < 50; ++i) {
        Timer::Evaluate([]() {
            volatile double sum = 0;
            for (int j = 0; j < 100000; ++j) {
                sum += std::sin(j);
            }
            (void)sum;
        }, "heavy_compute");
    }

    double avg = Timer::GetMeanTime("heavy_compute");
    std::cout << "  50 calls, mean: " << avg << " ms" << std::endl;

    Timer::PrintAll();
}

void test_timer_dump_csv() {
    std::cout << "=== test_timer_dump_csv ===" << std::endl;
    Timer::Clear();

    Timer::Evaluate([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }, "task_a");

    Timer::Evaluate([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }, "task_b");

    Timer::DumpIntoFile("/tmp/timer_test.csv");
    std::cout << "  check /tmp/timer_test.csv" << std::endl;
}

// ============================================================
//  AsyncMessageProcess tests
// ============================================================

void test_async_basic() {
    std::cout << "=== test_async_basic ===" << std::endl;
    Timer::Clear();

    std::atomic<int> count{0};

    AsyncMessageProcess<int> proc([&count](const int& msg) {
        Timer::Evaluate([&count, &msg]() {
            count += msg;
        }, "callback_add");
    }, "adder");

    proc.Start();

    for (int i = 1; i <= 100; ++i) {
        proc.AddMessage(i);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    proc.Quit();

    int expected = 100 * 101 / 2;
    std::cout << "  sum = " << count << " (expect " << expected << ")" << std::endl;
    Timer::PrintAll();
}

void test_async_ordering() {
    std::cout << "=== test_async_ordering ===" << std::endl;
    Timer::Clear();

    std::vector<std::string> received;

    AsyncMessageProcess<std::string> proc([&received](const std::string& msg) {
        Timer::Evaluate([&received, &msg]() {
            received.push_back(msg);
        }, "callback_push");
    }, "order_test");

    proc.Start();

    std::vector<std::string> sent;
    for (int i = 0; i < 20; ++i) {
        std::string s = "msg_" + std::to_string(i);
        sent.push_back(s);
        proc.AddMessage(s);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    proc.Quit();

    bool ordered = (received == sent);
    std::cout << "  messages ordered: " << (ordered ? "YES" : "NO") << std::endl;
    std::cout << "  sent " << sent.size() << ", received " << received.size() << std::endl;
    Timer::PrintAll();
}

void test_async_skip() {
    std::cout << "=== test_async_skip ===" << std::endl;
    Timer::Clear();

    std::atomic<int> count{0};

    AsyncMessageProcess<int> proc([&count](const int& msg) {
        Timer::Evaluate([&count]() {
            count++;
        }, "callback_inc");
    }, "skip_test");

    proc.SetSkipParam(true, 3);
    proc.Start();

    for (int i = 0; i < 30; ++i) {
        proc.AddMessage(i);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    proc.Quit();

    std::cout << "  sent 30, processed " << count << " (expect ~10 with skip=3)" << std::endl;
    Timer::PrintAll();
}

void test_async_max_size() {
    std::cout << "=== test_async_max_size ===" << std::endl;
    Timer::Clear();

    std::atomic<int> count{0};

    AsyncMessageProcess<int> proc([&count](const int& msg) {
        Timer::Evaluate([&count]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            count++;
        }, "callback_slow");
    }, "overflow_test");

    proc.SetMaxSize(5);
    proc.Start();

    for (int i = 0; i < 50; ++i) {
        proc.AddMessage(i);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    proc.Quit();

    std::cout << "  sent 50 with max_size=5, processed " << count << std::endl;
    Timer::PrintAll();
}

void test_async_multi_producer() {
    std::cout << "=== test_async_multi_producer ===" << std::endl;
    Timer::Clear();

    std::atomic<int> total{0};

    AsyncMessageProcess<int> proc([&total](const int& msg) {
        Timer::Evaluate([&total, &msg]() {
            total += msg;
        }, "callback_sum");
    }, "multi_prod");

    proc.Start();

    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&proc]() {
            for (int i = 1; i <= 25; ++i) {
                proc.AddMessage(i);
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    proc.Quit();

    int expected = 4 * (25 * 26 / 2);
    std::cout << "  sum = " << total << " (expect " << expected << ")" << std::endl;
    Timer::PrintAll();
}

// ============================================================

int main() {
    std::cout << "====== utils test suite ======\n" << std::endl;

    test_timer_basic();
    std::cout << std::endl;

    test_timer_multiple_calls();
    std::cout << std::endl;

    test_timer_dump_csv();
    std::cout << std::endl;

    test_async_basic();
    std::cout << std::endl;

    test_async_ordering();
    std::cout << std::endl;

    test_async_skip();
    std::cout << std::endl;

    test_async_max_size();
    std::cout << std::endl;

    test_async_multi_producer();
    std::cout << std::endl;

    std::cout << "====== all tests done ======" << std::endl;
    return 0;
}
