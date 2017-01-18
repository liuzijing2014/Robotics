"""
Zijing Liu 445 - Programming Assignment 1
"""

def mysum1(inputs):
    """
    Return the sum of integers in inputs

    Argument:
        inputs: a list of integers
    """
    result = 0
    for i in inputs:
        result += i
    return result

def myfib1(n):
    """
    Return nth Fibonacci number

    Arguments:
        n: the nth target
    """
    if n <= 1:
        return 1
    else:
        return myfib1(n-1) + myfib1(n-2)

if __name__ == "__main__":
    # TEST
    # mysum1 tests
    print(mysum1([1, 5, 7]))
    print(mysum1([]))
    print(mysum1([-5, 3]))

    # myfib1 tests
    print(myfib1(3))
    print(myfib1(5))
    print(myfib1(7))
