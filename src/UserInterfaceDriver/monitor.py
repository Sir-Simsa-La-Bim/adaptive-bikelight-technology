from driver import *
from cancel_token.token import CancelToken

_DEFAULT_REFRESH_INTERVAL = 1.0

async def runMonitor(port: str, cancelToken: CancelToken|None = None, refreshInterval: float = _DEFAULT_REFRESH_INTERVAL)->None:
    synchronizer = None
    try:
        synchronizer = ParameterSynchronizer(port, autoRefreshInterval=refreshInterval)
        parameters = [synchronizer.add(parameter) for parameter in PARAMETERS]

        while cancelToken is None or not cancelToken.triggered:
            s = '\033[3J\033[H\33[2J' # This sequence clears the console
            s += 'STATUS\n'
            if synchronizer.isConnected:
                s += 'connected'
            else:
                s += 'disconnected'
            s += f' ({port})'
            s += '\n\n'

            s += 'PARAMETERS\n'

            showValues = synchronizer.isConnected 
            for parameter in parameters:
                s += '- '
                s += parameter._parameter.name
                s += ': '
                value = parameter.getValue()
                if showValues and value is not None:
                    s += repr(value)
                s += '\n'
            s += '\n'
            print(s)

            try:
                async with asyncio.timeout(2 * refreshInterval):
                    await synchronizer.waitForFullSync()
            except TimeoutError:
                pass

    finally:
        if synchronizer is not None:
            synchronizer.close()

def runMonitorForever(port: str, refreshInterval: float = _DEFAULT_REFRESH_INTERVAL)->None:
    import asyncio
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(runMonitor(port, refreshInterval=refreshInterval))

if __name__ == '__main__':
    port = input('Run monitor on port: ')
    runMonitorForever(port)