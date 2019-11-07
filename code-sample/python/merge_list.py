def merge_lists(list_a, list_b):
    if not list_a:
        return list_b
    if not list_b:
        return list_a
    a = list_a[0]
    b = list_b[0]
    if a < b:
        return [a, *merge_lists(list_a[1:], list_b)]
    else:
        return [b, *merge_lists(list_a, list_b[1:])]


if __name__ == '__main__':
    list_a = [1, 3, 6, 2342]
    list_b = [3, 9, 10, 23]
    list_c = merge_lists(list_a, list_b)
    print(list_a)
    print(list_b)
    print(list_c)
