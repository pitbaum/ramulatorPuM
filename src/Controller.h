#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <unordered_map>

#include <cassert>
#include <cstdio>
#include <deque>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "Config.h"
#include "DRAM.h"
#include "Refresh.h"
#include "Request.h"
#include "Scheduler.h"
#include "Statistics.h"

#include "ALDRAM.h"
#include "SALP.h"
#include "TLDRAM.h"
#include "DDR4.h"

using namespace std;

namespace ramulator
{

    extern bool warmup_complete;

template <typename T>
class Controller
{
protected:
    // For counting bandwidth
    ScalarStat read_transaction_bytes;
    ScalarStat write_transaction_bytes;

    ScalarStat row_hits;
    ScalarStat row_misses;
    ScalarStat row_conflicts;
    VectorStat read_row_hits;
    VectorStat read_row_misses;
    VectorStat read_row_conflicts;
    VectorStat write_row_hits;
    VectorStat write_row_misses;
    VectorStat write_row_conflicts;
    ScalarStat useless_activates;

    ScalarStat read_latency_avg;
    ScalarStat read_latency_sum;

    ScalarStat req_queue_length_avg;
    ScalarStat req_queue_length_sum;
    ScalarStat read_req_queue_length_avg;
    ScalarStat read_req_queue_length_sum;
    ScalarStat write_req_queue_length_avg;
    ScalarStat write_req_queue_length_sum;
    ScalarStat rc_req_queue_length_avg;
    ScalarStat rc_req_queue_length_sum;
    ScalarStat maj_req_queue_length_avg;
    ScalarStat maj_req_queue_length_sum;

    ScalarStat rc_commands;
    ScalarStat maj_commands;
    ScalarStat frac_commands;
    

#ifndef INTEGRATED_WITH_GEM5
    VectorStat record_read_hits;
    VectorStat record_read_misses;
    VectorStat record_read_conflicts;
    VectorStat record_write_hits;
    VectorStat record_write_misses;
    VectorStat record_write_conflicts;
#endif

public:
    /* Member Variables */
    long clk = 0;
    DRAM<T>* channel;

    Scheduler<T>* scheduler;  // determines the highest priority request whose commands will be issued
    RowPolicy<T>* rowpolicy;  // determines the row-policy (e.g., closed-row vs. open-row)
    RowTable<T>* rowtable;  // tracks metadata about rows (e.g., which are open and for how long)
    Refresh<T>* refresh;

    struct Queue {
        list<Request> q;
        unsigned int max = 128; //originally was at 32
        unsigned int size() {return q.size();}
    };

    Queue readq;  // queue for read requests
    Queue writeq;  // queue for write requests
    Queue actq; // read, write and PuM requests for which activate was issued are moved to 
                   // actq, which has higher priority than readq and writeq.
                   // This is an optimization
                   // for avoiding useless activations (i.e., PRECHARGE
                   // after ACTIVATE w/o READ of WRITE command)
    Queue otherq;  // queue for all "other" requests (e.g., refresh)
    Queue rowcloneq; // queue for row clone requests
    Queue majq;  // queue for maj requests

    deque<Request> pending;  // read requests that are about to receive data from DRAM
    bool write_mode = false;  // whether write requests should be prioritized over reads
    float wr_high_watermark = 0.8f; // threshold for switching to write mode
    float wr_low_watermark = 0.2f; // threshold for switching back to read mode
    //long refreshed = 0;  // last time refresh requests were generated

    /* Command trace for DRAMPower 3.1 */
    string cmd_trace_prefix = "cmd-trace-";
    vector<ofstream> cmd_trace_files;
    bool record_cmd_trace = false;
    /* Commands to stdout */
    bool print_cmd_trace = false;

    /* Constructor */
    Controller(const Config& configs, DRAM<T>* channel) :
        channel(channel),
        scheduler(new Scheduler<T>(this)),
        rowpolicy(new RowPolicy<T>(this)),
        rowtable(new RowTable<T>(this)),
        refresh(new Refresh<T>(this)),
        cmd_trace_files(channel->children.size())
    {
        record_cmd_trace = configs.record_cmd_trace();
        print_cmd_trace = configs.print_cmd_trace();
        if (record_cmd_trace){
            if (configs["cmd_trace_prefix"] != "") {
              cmd_trace_prefix = configs["cmd_trace_prefix"];
            }
            string prefix = cmd_trace_prefix + "chan-" + to_string(channel->id) + "-rank-";
            string suffix = ".cmdtrace";
            for (unsigned int i = 0; i < channel->children.size(); i++)
                cmd_trace_files[i].open(prefix + to_string(i) + suffix);
        }

        // regStats

        row_hits
            .name("row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits per channel per core")
            .precision(0)
            ;
        row_misses
            .name("row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses per channel per core")
            .precision(0)
            ;
        row_conflicts
            .name("row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts per channel per core")
            .precision(0)
            ;

        read_row_hits
            .init(configs.get_core_num())
            .name("read_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for read requests per channel per core")
            .precision(0)
            ;
        read_row_misses
            .init(configs.get_core_num())
            .name("read_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for read requests per channel per core")
            .precision(0)
            ;
        read_row_conflicts
            .init(configs.get_core_num())
            .name("read_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for read requests per channel per core")
            .precision(0)
            ;

        write_row_hits
            .init(configs.get_core_num())
            .name("write_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for write requests per channel per core")
            .precision(0)
            ;
        write_row_misses
            .init(configs.get_core_num())
            .name("write_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for write requests per channel per core")
            .precision(0)
            ;
        write_row_conflicts
            .init(configs.get_core_num())
            .name("write_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for write requests per channel per core")
            .precision(0)
            ;

        useless_activates
            .name("useless_activates_"+to_string(channel->id)+ "_core")
            .desc("Number of useless activations. E.g, ACT -> PRE w/o RD or WR")
            .precision(0)
            ;

        read_transaction_bytes
            .name("read_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of read transaction per channel")
            .precision(0)
            ;
        write_transaction_bytes
            .name("write_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of write transaction per channel")
            .precision(0)
            ;

        read_latency_sum
            .name("read_latency_sum_"+to_string(channel->id))
            .desc("The memory latency cycles (in memory time domain) sum for all read requests in this channel")
            .precision(0)
            ;
        read_latency_avg
            .name("read_latency_avg_"+to_string(channel->id))
            .desc("The average memory latency cycles (in memory time domain) per request for all read requests in this channel")
            .precision(6)
            ;

        req_queue_length_sum
            .name("req_queue_length_sum_"+to_string(channel->id))
            .desc("Sum of read and write queue length per memory cycle per channel.")
            .precision(0)
            ;
        req_queue_length_avg
            .name("req_queue_length_avg_"+to_string(channel->id))
            .desc("Average of read and write queue length per memory cycle per channel.")
            .precision(6)
            ;

        read_req_queue_length_sum
            .name("read_req_queue_length_sum_"+to_string(channel->id))
            .desc("Read queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        read_req_queue_length_avg
            .name("read_req_queue_length_avg_"+to_string(channel->id))
            .desc("Read queue length average per memory cycle per channel.")
            .precision(6)
            ;

        write_req_queue_length_sum
            .name("write_req_queue_length_sum_"+to_string(channel->id))
            .desc("Write queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        write_req_queue_length_avg
            .name("write_req_queue_length_avg_"+to_string(channel->id))
            .desc("Write queue length average per memory cycle per channel.")
            .precision(6)
            ;

        

#ifndef INTEGRATED_WITH_GEM5
        record_read_hits
            .init(configs.get_core_num())
            .name("record_read_hits")
            .desc("record read hit count for this core when it reaches request limit or to the end")
            ;

        record_read_misses
            .init(configs.get_core_num())
            .name("record_read_misses")
            .desc("record_read_miss count for this core when it reaches request limit or to the end")
            ;

        record_read_conflicts
            .init(configs.get_core_num())
            .name("record_read_conflicts")
            .desc("record read conflict count for this core when it reaches request limit or to the end")
            ;

        record_write_hits
            .init(configs.get_core_num())
            .name("record_write_hits")
            .desc("record write hit count for this core when it reaches request limit or to the end")
            ;

        record_write_misses
            .init(configs.get_core_num())
            .name("record_write_misses")
            .desc("record write miss count for this core when it reaches request limit or to the end")
            ;

        record_write_conflicts
            .init(configs.get_core_num())
            .name("record_write_conflicts")
            .desc("record write conflict for this core when it reaches request limit or to the end")
            ;
#endif
        rc_commands
            .name("rc_commands_channel_" + to_string(channel->id))
            .desc("Number of RC (RowClone) commands issued per channel")
            .precision(0);
        maj_commands
            .name("maj_commands_channel_" + to_string(channel->id))
            .desc("Number of MAJ (Majority) commands issued per channel")
            .precision(0);
        frac_commands
            .name("frac_commands_channel_" + to_string(channel->id))
            .desc("Number of FRAC (Fractional) commands issued per channel")
            .precision(0);
    }

    ~Controller(){
        delete scheduler;
        delete rowpolicy;
        delete rowtable;
        delete channel;
        delete refresh;
        for (auto& file : cmd_trace_files)
            file.close();
        cmd_trace_files.clear();
    }

    void finish(long read_req, long dram_cycles) {
      read_latency_avg = read_latency_sum.value() / read_req;
      req_queue_length_avg = req_queue_length_sum.value() / dram_cycles;
      read_req_queue_length_avg = read_req_queue_length_sum.value() / dram_cycles;
      write_req_queue_length_avg = write_req_queue_length_sum.value() / dram_cycles;
      // call finish function of each channel
      channel->finish(dram_cycles);
    }

    /* Member Functions */
    Queue& get_queue(Request::Type type)
    {
        switch (int(type)) {
            case int(Request::Type::READ): return readq;
            case int(Request::Type::WRITE): return writeq;
            // Add the new command type to its own dedicated queue
            case int(Request::Type::RC): return rowcloneq;
            case int(Request::Type::MAJ): return majq;
            default: return otherq;
        }
    }

    // No further modifications necessary
    // Small optimization possible where a read can be issued from a PuM command
    // Analogue to the read write works to the same address
    bool enqueue(Request& req)
    {
        Queue& queue = get_queue(req.type);
        if (queue.max == queue.size())
            return false;

        req.arrive = clk;
        queue.q.push_back(req);
        // shortcut for read requests, if a write to same addr exists
        // necessary for coherence
        if (req.type == Request::Type::READ && find_if(writeq.q.begin(), writeq.q.end(),
                [req](Request& wreq){ return req.addr == wreq.addr;}) != writeq.q.end()){
            req.depart = clk + 1;
            pending.push_back(req);
            readq.q.pop_back();
        }
        return true;
    }

    void tick()
    {
        clk++;
        // shouldnt it be = rather than += ???
        req_queue_length_sum += readq.size() + writeq.size() + rowcloneq.size() + majq.size() + pending.size();
        read_req_queue_length_sum += readq.size() + pending.size();
        write_req_queue_length_sum += writeq.size();
        rc_req_queue_length_sum += rowcloneq.size();
        maj_req_queue_length_sum += majq.size();

        // Normally know the hierachy will be to first serve shortcut Reads,
        // then check the read and write queues if we do read or write command issueing
        // Instead prioritize PuM commands if all 32 packages for a request have arrived

        // Issues shortcut reads
        /*** 1. Serve completed reads ***/
        if (pending.size()) {
            Request& req = pending[0];
            if (req.depart <= clk) {
                if (req.depart - req.arrive > 1) { // this request really accessed a row
                  read_latency_sum += req.depart - req.arrive;
                  channel->update_serving_requests(
                      req.addr_vec.data(), -1, clk);
                }
                req.callback(req);
                pending.pop_front();
            }
        }

        /*** 2. Refresh scheduler ***/
        refresh->tick_ref();

        /*** 3. Should we schedule writes? ***/
        if (!write_mode) {
            // yes -- write queue is almost full or read queue is empty
            if (writeq.size() > int(wr_high_watermark * writeq.max) || readq.size() == 0)
                write_mode = true;
        }
        else {
            // no -- write queue is almost empty and read queue is not empty
            if (writeq.size() < int(wr_low_watermark * writeq.max) && readq.size() != 0)
                write_mode = false;
        }

        /*** 4. Find the best command to schedule, if any ***/

        // First check the actq (which has higher priority) to see if there
        // are requests available to service in this cycle
        Queue* queue = &actq;
        typename T::Command cmd;
        auto req = scheduler->get_head(queue->q);

        bool is_valid_req = (req != queue->q.end());

        if(is_valid_req) {
            cmd = get_first_cmd(req);
            is_valid_req = is_ready(cmd, req->addr_vec);
        }

        if (!is_valid_req) {
            queue = !write_mode ? &readq : &writeq;

            /*
                Do PuM related scheduling that has precedence over the read write stuff
            */

            // Conditionals when and how to check for a ready to promote PuM request
            if (rowcloneq.size() >= 32 || majq.size() >= 32) {
                bool is_rowclone_ready = rowcloneq.size() >= 32 && has_any_group_of_32_same_addr(rowcloneq);
                bool is_maj_ready = majq.size() >= 32 && has_any_group_of_32_same_addr(majq);
                // If a request in maj and rowclone are both ready
                // Put them in order of arrival into the otherq 
                if (is_rowclone_ready && is_maj_ready) {
                    auto rowclone_head = rowcloneq.q.front();
                    auto maj_head = majq.q.front();
                    if (rowclone_head.arrive < maj_head.arrive) {
                        put_pum_into_queue(rowcloneq, otherq.q);
                        // Put 2 x 2 frac commands into the otherq in front of the maj
                        put_pum_into_queue(majq, otherq.q);
                    } else {
                        // Put two frac commands into the otherq in front of the maj
                        put_pum_into_queue(majq, otherq.q);
                        put_pum_into_queue(rowcloneq, otherq.q);
                    }
                } else if (is_rowclone_ready) {
                    put_pum_into_queue(rowcloneq, otherq.q);
                } else if (is_maj_ready) {

                    // Put two frac commands into the otherq in front of the maj
                    put_pum_into_queue(majq, otherq.q);
                } // In case none of them are ready, dont do anything
            }

            /*
                Continue operation as usual, if PuM was ready, added it to otherq to be issued with priority
            */

            // Other requests
            if (otherq.size())
                queue = &otherq;  // "other" requests are rare, so we give them precedence over reads/writes

            req = scheduler->get_head(queue->q);

            is_valid_req = (req != queue->q.end());

            if(is_valid_req){
                cmd = get_first_cmd(req);
                is_valid_req = is_ready(cmd, req->addr_vec);
            }
        }

        if (!is_valid_req) {
            // we couldn't find a command to schedule -- let's try to be speculative
            auto cmd = T::Command::PRE;
            vector<int> victim = rowpolicy->get_victim(cmd);
            if (!victim.empty()){
                issue_cmd(cmd, victim);
            }
            return;  // nothing more to be done this cycle
        }

        // Updates several stats, potential to check if the PuM request was a hit eventough hits are not allowed
        if (req->is_first_command) {
            req->is_first_command = false;
            int coreid = req->coreid;
            if (req->type == Request::Type::READ || req->type == Request::Type::WRITE) {
              channel->update_serving_requests(req->addr_vec.data(), 1, clk);
            }
            int tx = (channel->spec->prefetch_size * channel->spec->channel_width / 8);
            if (req->type == Request::Type::READ) {
                if (is_row_hit(req)) {
                    ++read_row_hits[coreid];
                    ++row_hits;
                } else if (is_row_open(req)) {
                    ++read_row_conflicts[coreid];
                    ++row_conflicts;
                } else {
                    ++read_row_misses[coreid];
                    ++row_misses;
                }
              read_transaction_bytes += tx;
            } else if (req->type == Request::Type::WRITE) {
              if (is_row_hit(req)) {
                  ++write_row_hits[coreid];
                  ++row_hits;
              } else if (is_row_open(req)) {
                  ++write_row_conflicts[coreid];
                  ++row_conflicts;
              } else {
                  ++write_row_misses[coreid];
                  ++row_misses;
              }
              write_transaction_bytes += tx;
            }
        }

        // issue command on behalf of request
        issue_cmd(cmd, get_addr_vec(cmd, req));

        // check whether this is the last command (which finishes the request)
        //if (cmd != channel->spec->translate[int(req->type)]){
        if (cmd != channel->spec->translate[int(req->type)]) {
            if(channel->spec->is_opening(cmd)) {
                // promote the request that caused issuing activation to actq
                actq.q.push_back(*req);
                queue->q.erase(req);
            }

            return;
        }

        // set a future completion time for read requests
        if (req->type == Request::Type::READ) {
            req->depart = clk + channel->spec->read_latency;
            pending.push_back(*req);
        }

        if (req->type == Request::Type::WRITE) {
            channel->update_serving_requests(req->addr_vec.data(), -1, clk);
            // req->callback(*req);
        }

        // remove request from queue
        queue->q.erase(req);
    }

    bool is_ready(list<Request>::iterator req)
    {
        typename T::Command cmd = get_first_cmd(req);
        return channel->check(cmd, req->addr_vec.data(), clk);
    }

    bool is_ready(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check(cmd, addr_vec.data(), clk);
    }

    bool is_row_hit(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_hit(cmd, req->addr_vec.data());
    }

    bool is_row_hit(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_hit(cmd, addr_vec.data());
    }

    bool is_row_open(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_open(cmd, req->addr_vec.data());
    }

    bool is_row_open(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_open(cmd, addr_vec.data());
    }

    void update_temp(ALDRAM::Temp current_temperature)
    {
    }

    // For telling whether this channel is busying in processing read or write
    bool is_active() {
      return (channel->cur_serving_requests > 0);
    }

    // For telling whether this channel is under refresh
    bool is_refresh() {
      return clk <= channel->end_of_refreshing;
    }

    void set_high_writeq_watermark(const float watermark) {
       wr_high_watermark = watermark; 
    }

    void set_low_writeq_watermark(const float watermark) {
       wr_low_watermark = watermark;
    }

    void record_core(int coreid) {
#ifndef INTEGRATED_WITH_GEM5
      record_read_hits[coreid] = read_row_hits[coreid];
      record_read_misses[coreid] = read_row_misses[coreid];
      record_read_conflicts[coreid] = read_row_conflicts[coreid];
      record_write_hits[coreid] = write_row_hits[coreid];
      record_write_misses[coreid] = write_row_misses[coreid];
      record_write_conflicts[coreid] = write_row_conflicts[coreid];
#endif
    }

private:
    typename T::Command get_first_cmd(list<Request>::iterator req)
    {
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->decode(cmd, req->addr_vec.data());
    }

    // Hash for vector<int> (To check for same addresses in a queue)
    // In reality would need to check for same tag of parent request, but request in ramulator doesnt have such a field
    struct VectorHash {
        std::size_t operator()(const std::vector<int>& vec) const {
            std::size_t seed = vec.size();
            for (int v : vec) {
                seed ^= std::hash<int>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    // Check if 32 requests have the same addr_vec in a queue
    bool has_any_group_of_32_same_addr(const Queue& pQueue) {
        std::unordered_map<std::vector<int>, int, VectorHash> addr_vec_counts;
        for (const auto& req : pQueue.q) {
            ++addr_vec_counts[req.addr_vec];
            if (addr_vec_counts[req.addr_vec] >= 32) {
                return true;
            }
        }
        return false;
    }

    // Get first 32 requests with matching addr_vec
    std::list<Request>::iterator find_common_group_start(Queue& pQueue, std::vector<int>& out_vec) {
        std::unordered_map<std::vector<int>, int, VectorHash> addr_vec_counts;
        for (const auto& req : pQueue.q) {
            ++addr_vec_counts[req.addr_vec];
        }
        for (const auto& [vec, count] : addr_vec_counts) {
            if (count >= 32) {
                out_vec = vec;
                break;
            }
        }
        if (out_vec.empty()) return pQueue.q.end();

        // Find the first iterator where a group of 32 starts
        int found = 0;
        for (auto it = pQueue.q.begin(); it != pQueue.q.end(); ++it) {
            if (it->addr_vec == out_vec) {
                ++found;
                if (found == 1) return it;  // return first match
            }
        }
        return pQueue.q.end();
    }

            // Moves first matching to `otherq` as first, remove the rest from the queue
    void put_pum_into_queue(Queue& pQueue, std::list<Request>& otherq) {
        std::vector<int> match_vec;
        auto it = find_common_group_start(pQueue, match_vec);
        if (it == pQueue.q.end()) return;

        // Promote first request
        it->is_first_command = true;

        Request main_req = std::move(*it);
        it = pQueue.q.erase(it);

        // Remove other 31 with the same addr_vec
        int removed = 1;
        for (auto iter = pQueue.q.begin(); iter != pQueue.q.end() && removed < 32; ) {
            if (iter->addr_vec == match_vec) {
                iter = pQueue.q.erase(iter);
                ++removed;
            } else {
                ++iter;
            }
        }

        // If this is a MAJ request, insert 4 FRACs before it
        if (main_req.type == Request::Type::MAJ) {
            Request frac1 = std::move(main_req);  // Move main_req to frac1
            Request frac2 = frac1; // Copy frac1 (should be safe if callback is moved only once)
            Request frac3 = frac1;
            Request frac4 = frac1;

            frac1.type = Request::Type::FRAC;
            frac1.is_first_command = true;
            frac2.type = Request::Type::FRAC;
            frac2.is_first_command = false;
            frac3.type = Request::Type::FRAC;
            frac3.is_first_command = false;
            frac4.type = Request::Type::FRAC;
            frac4.is_first_command = false;

            otherq.push_back(std::move(frac1));
            otherq.push_back(std::move(frac2));
            otherq.push_back(std::move(frac3));
            otherq.push_back(std::move(frac4));
        }

        otherq.push_back(std::move(main_req));
    }


    // upgrade to an autoprecharge command
    void cmd_issue_autoprecharge(typename T::Command& cmd,
                                            const vector<int>& addr_vec) {

        // currently, autoprecharge is only used with closed row policy
        if(channel->spec->is_accessing(cmd) && rowpolicy->type == RowPolicy<T>::Type::ClosedAP) {
            // check if it is the last request to the opened row
            Queue* queue = write_mode ? &writeq : &readq;

            auto begin = addr_vec.begin();
            vector<int> rowgroup(begin, begin + int(T::Level::Row) + 1);

			int num_row_hits = 0;

            for (auto itr = queue->q.begin(); itr != queue->q.end(); ++itr) {
                if (is_row_hit(itr)) { 
                    auto begin2 = itr->addr_vec.begin();
                    vector<int> rowgroup2(begin2, begin2 + int(T::Level::Row) + 1);
                    if(rowgroup == rowgroup2)
                        num_row_hits++;
                }
            }

            if(num_row_hits == 0) {
                Queue* queue = &actq;
                for (auto itr = queue->q.begin(); itr != queue->q.end(); ++itr) {
                    if (is_row_hit(itr)) {
                        auto begin2 = itr->addr_vec.begin();
                        vector<int> rowgroup2(begin2, begin2 + int(T::Level::Row) + 1);
                        if(rowgroup == rowgroup2)
                            num_row_hits++;
                    }
                }
            }

            assert(num_row_hits > 0); // The current request should be a hit, 
                                      // so there should be at least one request 
                                      // that hits in the current open row
            if(num_row_hits == 1) {
                if(cmd == T::Command::RD)
                    cmd = T::Command::RDA;
                else if (cmd == T::Command::WR)
                    cmd = T::Command::WRA;
                else
                    assert(false && "Unimplemented command type.");
            }
        }

    }

    void issue_cmd(typename T::Command cmd, const vector<int>& addr_vec)
    {
        cmd_issue_autoprecharge(cmd, addr_vec);
        assert(is_ready(cmd, addr_vec));
        channel->update(cmd, addr_vec.data(), clk);

        if(cmd == T::Command::PRE){
            if(rowtable->get_hits(addr_vec, true) == 0){
                useless_activates++;
            }
        }
 
        rowtable->update(cmd, addr_vec, clk);
        if (record_cmd_trace){
            // select rank
            auto& file = cmd_trace_files[addr_vec[1]];
            string& cmd_name = channel->spec->command_name[int(cmd)];
            file<<clk<<','<<cmd_name;
            // TODO bad coding here
            if (cmd_name == "PREA" || cmd_name == "REF")
                file<<endl;
            else{
                int bank_id = addr_vec[int(T::Level::Bank)];
                if (channel->spec->standard_name == "DDR4" || channel->spec->standard_name == "GDDR5")
                    bank_id += addr_vec[int(T::Level::Bank) - 1] * channel->spec->org_entry.count[int(T::Level::Bank)];
                file<<','<<bank_id<<endl;
            }
        }
        if (print_cmd_trace){
            printf("%5s %10ld:", channel->spec->command_name[int(cmd)].c_str(), clk);
            for (int lev = 0; lev < int(T::Level::MAX); lev++)
                printf(" %5d", addr_vec[lev]);
            printf("\n");
        }
    }
    vector<int> get_addr_vec(typename T::Command cmd, list<Request>::iterator req){
        return req->addr_vec;
    }
};

template <>
vector<int> Controller<SALP>::get_addr_vec(
    SALP::Command cmd, list<Request>::iterator req);

template <>
bool Controller<SALP>::is_ready(list<Request>::iterator req);

template <>
void Controller<ALDRAM>::update_temp(ALDRAM::Temp current_temperature);

template <>
void Controller<TLDRAM>::tick();

template <>
void Controller<TLDRAM>::cmd_issue_autoprecharge(typename TLDRAM::Command& cmd,
                                                    const vector<int>& addr_vec);

} /*namespace ramulator*/

#endif /*__CONTROLLER_H*/
