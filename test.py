"""

"""


def checkIPValidity(addressIP):
    # Write your code here
    res = None
    arr = addressIP.split('.')
    for i in range(arr):
        a = int(arr[i])
        if a < 0 or a > 255:
            return "INVALID"

    return "VALID"


def main():
    # input for addressIP
    addressIP = str(input())

    result = checkIPValidity(addressIP)
    print(result)


if __name__ == "__main__":
    main()