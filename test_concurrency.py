import time
import requests
import multiprocessing

remote = False
if remote:
    ip = "192.168.1.12"
else:
    ip = "localhost"


def make_requests(ip, _):
    start_time = time.time()
    total = 10
    total_count = 0
    for i in range(total):
        try:
            # r = requests.get(
            #     f"http://{ip}/",
            #     timeout=10,
            # )
            # print(_, "fourB")
            # total_count += 1
            # r = requests.get(
            #     f"http://{ip}/api/v1/observingconditions/0/winddirection",
            #     timeout=1,
            # )
            r = requests.get(
                f"http://localhost:8000/api/db/polling/ObservingConditions?day=0.1",
                timeout=1,
            )
            # print(_, "winddirection", r.json())
            total_count += 1

            # r = requests.get(
            #     f"http://{ip}/api/v1/observingconditions/0/skytemperature",
            #     timeout=10,
            # )
            # print(_, "skytemperature", r.json()["Value"])
            # total_count += 1

        except requests.exceptions.RequestException as e:
            print(e)

    print("--- %.1f ms ---" % (1000 * (time.time() - start_time) / total_count))


if __name__ == "__main__":
    processes = []
    n_clients = 4
    for _ in range(n_clients):
        p = multiprocessing.Process(
            target=make_requests,
            args=(
                ip,
                _,
            ),
        )
        processes.append(p)
        p.start()

    for p in processes:
        p.join()
