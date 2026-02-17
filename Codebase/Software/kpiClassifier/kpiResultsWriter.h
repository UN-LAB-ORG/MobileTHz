#ifndef KPIRESULTSWRITER_H
#define KPIRESULTSWRITER_H

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "blockingconcurrentqueue.h"

struct WriteTask
{
    std::string filename;
    std::string content;
};

/**
 * @class KPIResultsWriter
 * @brief A thread-safe singleton class for asynchronous file writing.
 *
 * This class manages a dedicated thread to handle all file output,
 * preventing I/O operations from blocking the main application threads.
 * It uses a high-performance, lock-free blocking queue.
 */
class KPIResultsWriter
{
public:
    /**
     * @brief Gets the single instance of the KPIResultsWriter.
     * @return Reference to the singleton instance.
     */
    static KPIResultsWriter &getInstance()
    {
        static KPIResultsWriter instance;
        return instance;
    }

    /**
     * @brief Adds a single file writing task to the queue (lock-free).
     * @param filename The full path of the file to write.
     * @param content The string content to be written to the file.
     */
    void addWriteTask(const std::string &filename, const std::string &content)
    {
        // Enqueueing is a lock-free operation.
        writeQueue_.enqueue({filename, content});
    }

    /**
     * @brief Adds a batch of file writing tasks to the queue (lock-free).
     * @param tasks A vector of WriteTask objects to be added.
     */
    void addWriteTasks(std::vector<WriteTask> &&tasks)
    {
        // Use the move-iterator overload for maximum efficiency.
        writeQueue_.enqueue_bulk(std::make_move_iterator(tasks.begin()), tasks.size());
    }

    // Delete copy constructor and assignment operator to enforce singleton pattern.
    KPIResultsWriter(const KPIResultsWriter &) = delete;
    void operator=(const KPIResultsWriter &) = delete;

private:
    /**
     * @brief Private constructor to initialize the writer thread.
     */
    KPIResultsWriter()
        : stop_(false)
    {
        writerThread_ = std::thread(&KPIResultsWriter::processQueue, this);
    }

    /**
     * @brief Destructor to gracefully shut down the writer thread.
     */
    ~KPIResultsWriter()
    {
        // Signal the thread to stop.
        stop_.store(true);

        // Enqueue a dummy task to wake up the writer thread if it's blocked
        // waiting for an item. This is a robust way to ensure it checks the stop_ flag.
        writeQueue_.enqueue({"", ""});

        if (writerThread_.joinable())
        {
            writerThread_.join(); // Wait for the thread to finish completely.
        }
    }

    /**
     * @brief The main function for the dedicated writer thread using a blocking queue.
     */
    void processQueue()
    {
        WriteTask task;
        while (!stop_.load())
        {
            // This call will block efficiently (using a semaphore) until a task is available.
            // It will then populate 'task' with the dequeued item.
            writeQueue_.wait_dequeue(task);

            // If the dequeued task is the dummy task from the destructor, or if the stop
            // flag was set while we were waiting, we check the loop condition again.
            if (stop_.load() && task.filename.empty())
            {
                continue;
            }

            // Perform the file I/O operation. This happens outside any lock.
            try
            {
                // Ensure the target directory exists before writing.
                std::filesystem::path filePath(task.filename);
                std::filesystem::path dirPath = filePath.parent_path();

                if (!dirPath.empty() && !std::filesystem::exists(dirPath))
                {
                    std::filesystem::create_directories(dirPath);
                }

                std::ofstream outFile(task.filename);
                if (outFile.is_open())
                {
                    outFile << task.content;
                }
                else
                {
                    std::cerr << "Error [KPIResultsWriter]: Could not open file: " << task.filename
                              << std::endl;
                }
            }
            catch (const std::filesystem::filesystem_error &e)
            {
                std::cerr << "Filesystem Error [KPIResultsWriter]: " << e.what() << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "Exception [KPIResultsWriter]: " << e.what() << std::endl;
            }
        }
    }

    // --- Member variables for the lock-free implementation ---
    std::thread writerThread_;
    moodycamel::BlockingConcurrentQueue<WriteTask> writeQueue_;
    std::atomic<bool> stop_;
};

#endif // KPIRESULTSWRITER_H
