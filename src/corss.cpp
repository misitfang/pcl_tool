/**
 * c++提供了两种变量声明。
 * 一种是定义声明（defining declaration）简称定义，它给变量分配内存空间,
 * 另外一种是引用声明（referencing declaration）简称为声明，它不给变量分配空间，因为它引用已有变量。
 * 如果要在多个文件中使用外部变量，只需在一个文件中包含该变量的定义（ORD）,
 * 但在使用改变量的其他所有文件中，都必须使用extern声明这个变量。
 *
 * static	用于定义静态变量，表示该变量的作用域仅限于当前文件或当前函数内，不会被其他文件或函数访问。
 * static 修饰符也可以应用于全局变量。当 static 修饰全局变量时，会使变量的作用域限制在声明它的文件内
 *
 * ///位运算符号 整除时候优化时候可用于提速
 * <<	二进制左移运算符。将一个运算对象的各二进制位全部左移若干位（左边的二进制位丢弃，右边补0）。
 * <<2 表示将二进制位全部左移2位，相当于最后结果乘以4
 * >>2 表示右移2位，相当于除以4
 * <<n 表示左移n位，相当于乘以2^n
 *
 * 函数参数传入
 * 传值调用	该方法把参数的实际值赋值给函数的形式参数。在这种情况下，修改函数内的形式参数对实际参数没有影响。
 * 指针调用	该方法把参数的地址赋值给形式参数。在函数内，该地址用于访问调用中要用到的实际参数。
 * 这意味着，修改形式参数会影响实际参数
 *
 * 从函数中返回数组：
 *      C++ 不允许返回一个完整的数组作为函数的参数。但是，可以通过指定不带索引的数组名来返回一个指向数组的
 * 指针。
 * 如果想要从函数返回一个一维数组，您必须声明一个返回指针的函数。不能简单地返回指向局部数组的指针，因为当函数结
 * 束时，局部数组将被销毁，指向它的指针将变得无效。
 *      C++ 不支持在函数外返回局部变量的地址，除非定义局部变量为 static 变量。为了避免以上情况，你可以使用静态
 * 数组或者、动态分配数组。使用静态数组需要在函数内部创建一个静态数组，并将其地址返回，使用动态分配数组需要在函
 * 数内部使用new 运算符来分配一个数组，并在函数结束时使用 delete 运算符释放该数组。
 *
 *     指针：指针时一个变量，其值为另一个变量的地址，即内存位置的直接位置，必须在使用指针存储其他变量地址之前，
 * 对其进行声明。定义一个指针变量、把变量地址赋值给指针、访问指针变量中可用地址的值。这些是通过使用一元运算符 *
 *  来返回位于操作数所指定地址的变量的值.
 *      null指针 初始化指针时可以初始化为一个空指针null ，NULL 指针是一个定义在标准库中的值为零的常量。在大多
 * 数的操作系统上，不允许访问地址为 0 的内存，因为该内存是操作系统保留的。然而，内存地址 0 有特别重要的意义，它
 * 表明该指针不指向一个可访问的内存位
 * 指针与数组
 *     指针和数组在很多情况下是可以互换的。例如，一个指向数组开头的指针，可以通过使用指针的算术运算或数组索引来
 * 访问数组。
 *
 *  指针的指针就是将指针的地址存放在另一个指针里面。
 *
 *  引用变量是一个别名，也就是说，它是某个已存在变量的另一个名字。一旦把引用初始化为某个变量，
 * 就可以使用该引用名称或变量名称来指向变量。
 * 不存在空引用。引用必须连接到一块合法的内存。
 * 一旦引用被初始化为一个对象，就不能被指向到另一个对象。指针可以在任何时候指向到另一个对象。
 * 引用必须在创建时被初始化。指针可以在任何时间被初始化。
 *
 * 日期和时间：
 *  time_t now=time(NULL);获取当前系统当前时间
 *  tm currtime=*localtime(&now);返回一个指向表示本地时间的 tm 结构的指针
 *
 * strcpy():
 * 该strcpy()函数有两个参数：dest和src。它将src指向的字符串复制到dest指向的存储位置。
 * 空终止符也会被复制。
 *
 *
 * 类 定义一个类，本质上是定义一个数据类型的蓝图，它定义了类的对象包括了什么，以及可以在这个对象上执行哪些操作
 * 私有的成员和受保护的成员不能使用直接成员访问运算符 (.) 来直接访问
 *  类成员函数：类的成员函数是指那些把定义和原型写在类定义内部的函数，就像类定义中的其他变量一样。类成员函数是类的
 * 一个成员，它可以操作类的任意对象，可以访问对象中的所有成员。成员函数可以定义在类定义内部，或
 * 者单独使用范围解析运算符 :: 来定义,在类定义中定义的成员函数把函数声明为内联的，即便没有使用 inline 标识符.
 *  类访问修饰符: 类成员的访问限制是通过在类主体内部对各个区域标记 public、private、protected 来指定的。
 * 关键字 public、private、protected 称为访问修饰符。
 * 私有成员变量或函数在类的外部是不可访问的，甚至是不可查看的。只有类和友元函数可以访问私有成员。
 *
 * 继承中的特点
 *  有public, protected, private三种继承方式，它们相应地改变了基类成员的访问属性。
 *  1.public 继承：基类 public 成员，protected 成员，private 成员的访问属性在派生类中分别变成：
 *      public, protected, private
 *  2.protected 继承：基类 public 成员，protected 成员，private 成员的访问属性在派生类中分别变成：
 *      protected, protected, private
 *  3.private 继承：基类 public 成员，protected 成员，private 成员的访问属性在派生类中分别变成：
 *      private, private, private
 * 但无论哪种继承方式，上面两点都没有改变：
 *  1.private 成员只能被本类成员（类内）和友元访问，不能被派生类访问；
 *  2.protected 成员可以被派生类访问。
 *
 * 类的构造函数：
 *   1.类的构造函数是类的一种特殊的成员函数，它会在每次创建类的新对象时执行
 *   2.构造函数的名称与类的名称是完全相同的，并且不会返回任何类型，也不会返回 void。
 * 构造函数可用于为某些成员变量设置初始值。
 *
 * 类的析构函数: 是类的一种特殊的成员函数，它会在每次删除所创建的对象时执行
 *
 * 拷贝构造函数:
 * 是一种特殊的构造函数，它在创建对象时，是使用同一类中之前创建的对象来初始化新创建的对象。拷贝构造函数通常用于：
 *  1.通过使用另一个同类型的对象来初始化新创建的对象。
 *  2.复制对象把它作为参数传递给函数。
 *  3.复制对象，并从函数返回这个对象。
 * 如果在类中没有定义拷贝构造函数，编译器会自行定义一个。如果类带有指针变量，并有动态内存分配，则它必须有一个拷贝
 * 构造函数.
 *
 * 类的友元函数是定义在类外部，但有权访问类的所有私有（private）成员和保护（protected）成员。
 * 尽管友元函数的原型有在类的定义中出现过，但是友元函数并不是成员函数。友元可以是一个函数，该函数被称为友元函数；
 * 友元也可以是一个类，该类被称为友元类，在这种情况下，整个类及其所有成员都是友元。如果要声明函数为一个类的友元，
 * 需要在类定义中该函数原型前使用关键字 friend
 *
 * 类的友元函数:  是定义在类外部，但有权访问类的所有私有（private）成员和保护（protected）成员。尽管友元函数的
 * 原型有在类的定义中出现过，但是友元函数并不是成员函数。友元可以是一个函数，该函数被称为友元函数；友元也可以是一
 * 个类，该类被称为友元类，在这种情况下，整个类及其所有成员都是友元,如果要声明函数为一个类的友元，需要在类定义中
 * 该函数原型前使用关键字 friend
 *
 * 内联函数： 是通常与类一起使用。如果一个函数是内联的，那么在编译时，编译器会把该函数的代码副本放置在每个
 * 调用该函数的地方
 *
 *      this 是 C++ 中的一个关键字，也是一个 const 指针，它指向当前对象，通过它可以访问当前对象的所有成员。
 * this是一个隐藏的指针，可以在类的成员函数中使用，它可以用来指向调用对象,当一个对象的成员函数被调用时，
 * 编译器会隐式地传递该对象的地址作为 this 指针。友元函数没有 this 指针，因为友元不是类的成员，只有成员函数才有 this 指针
 *
 *
 *      静态成员在类的所有对象中是共享的。如果不存在其他的初始化语句，在创建第一个对象时，所有的静态数据都会被初始化为零。
 * 我们不能把静态成员的初始化放置在类的定义中，但是可以在类的外部通过使用范围解析运算符 :: 来重新声明静态变量从而对它进行
 * 初始化为零。我们不能把静态成员的初始化放置在类的定义中，但是可以在类的外部通过使用范围解析运算符 :: 来重新声明静态变量
 * 从而对它进行初始化
 *      静态成员函数与普通成员函数的区别：静态成员函数没有 this 指针，只能访问静态成员（包括静态成员变量和静态成员函数）。
 * 普通成员函数有 this 指针，可以访问类中的任意成员；而静态成员函数没有 this 指针
 * 如果把函数成员声明为静态的，就可以把函数与类的任何特定对象独立开来。静态成员函数即使在类对象不存在的情况下也能被调用，
 * 静态函数只要使用类名加范围解析运算符 :: 就可以访问。
 *
 *
 *      class derived-class: access-specifier base-class：
 * 其中，访问修饰符 access-specifier 是 public、protected 或 private 其中的一个，base-class 是之前定义过的某个
 * 类的名称。如果未使用访问修饰符 access-specifier，则默认为 private。
 *
 * 栈：在函数内部声明的所有变量都将占用栈内存。
 * 堆：这是程序中未使用的内存，在程序运行时可用于动态分配内存。
 *
 *      kdtree 对于一个由n维数据构成的数据集，我们首先寻找方差最大的那个维度，
 * 设这个维度是d，然后找出在维度d上所有数据项的中位数m，按m划分数据集，一分为二
 *
 *
 *  如果STL容器中的元素是Eigen库数据结构，例如这里定义一个vector容器，元素是Matrix4d
 * 如下所示vector<Eigen::Matrix4d>;这个错误也是和上述一样的提示，编译不会出错，只有在运行的时候出错。
 * 解决的方法很简单，定义改成下面的方式：vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>>;
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * */

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <limits>
#include <vector>
#include <fstream>
using namespace std;

void pointFunTest(int *a, int *b)
{
    // swap
    int tmp;
    tmp = *a;
    *a = *b;
    *b = tmp;
    return;
}

void cal_outer_product(const float vec1[3], const float vec2[3], float vecDst[3])
{
    vecDst[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2]; /*y1z2-y2z1*/
    vecDst[1] = vec2[0] * vec1[2] - vec1[0] * vec2[2]; /*x2z1-x1z2*/
    vecDst[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1]; /*x1y2-x2y1*/
}

std::ostream &operator<<(std::ostream &output, std::vector<int> arr)
{
    for (size_t i = 0; i < arr.size(); i++)
    {
        output << arr[i] << ",\n";
    }
    return output;
}
int main()
{

    std::cout << "type: \t\t"
              << "*\'***********size**************" << std::endl;
    std::cout << "bool: \t\t"
              << "所占字节数：" << sizeof(bool);
    std::cout << "\t最大值：" << (std::numeric_limits<bool>::max)();
    std::cout << "\t\t最小值：" << (std::numeric_limits<bool>::min)() << std::endl;
    std::cout << "char: \t\t"
              << "所占字节数：" << sizeof(char);
    std::cout << "\t最大值：" << (std::numeric_limits<char>::max)();
    std::cout << "\t\t\t最小值：" << (std::numeric_limits<char>::min)() << std::endl;
    std::cout << "signed char: \t"
              << "所占字节数：" << sizeof(signed char);
    std::cout << "\t最大值：" << (std::numeric_limits<signed char>::max)();
    std::cout << "\t\t\t最小值：" << (std::numeric_limits<signed char>::min)() << std::endl;
    std::cout << "unsigned char: \t"
              << "所占字节数：" << sizeof(unsigned char);
    std::cout << "\t最大值：" << (std::numeric_limits<unsigned char>::max)();
    std::cout << "\t\t最小值：" << (std::numeric_limits<unsigned char>::min)() << std::endl;
    std::cout << "wchar_t: \t"
              << "所占字节数：" << sizeof(wchar_t);
    std::cout << "\t最大值：" << (std::numeric_limits<wchar_t>::max)();
    std::cout << "\t\t最小值：" << (std::numeric_limits<wchar_t>::min)() << std::endl;
    std::cout << "short: \t\t"
              << "所占字节数：" << sizeof(short);
    std::cout << "\t最大值：" << (std::numeric_limits<short>::max)();
    std::cout << "\t\t最小值：" << (std::numeric_limits<short>::min)() << std::endl;
    std::cout << "short int : \t"
              << "所占字节数：" << sizeof(short int);
    std::cout << "\t最大值：" << (std::numeric_limits<short int>::max)();
    std::cout << "\t\t最小值：" << (std::numeric_limits<short int>::min)() << std::endl;
    std::cout << "int: \t\t"
              << "所占字节数：" << sizeof(int);
    std::cout << "\t最大值：" << (std::numeric_limits<int>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<int>::min)() << std::endl;
    std::cout << "unsigned: \t"
              << "所占字节数：" << sizeof(unsigned);
    std::cout << "\t最大值：" << (std::numeric_limits<unsigned>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<unsigned>::min)() << std::endl;
    std::cout << "long: \t\t"
              << "所占字节数：" << sizeof(long);
    std::cout << "\t最大值：" << (std::numeric_limits<long>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<long>::min)() << std::endl;
    std::cout << "unsigned long: \t"
              << "所占字节数：" << sizeof(unsigned long);
    std::cout << "\t最大值：" << (std::numeric_limits<unsigned long>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<unsigned long>::min)() << std::endl;
    std::cout << "double: \t"
              << "所占字节数：" << sizeof(double);
    std::cout << "\t最大值：" << (std::numeric_limits<double>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<double>::min)() << std::endl;
    std::cout << "long double: \t"
              << "所占字节数：" << sizeof(long double);
    std::cout << "\t最大值：" << (std::numeric_limits<long double>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<long double>::min)() << std::endl;
    std::cout << "float: \t\t"
              << "所占字节数：" << sizeof(float);
    std::cout << "\t最大值：" << (std::numeric_limits<float>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<float>::min)() << std::endl;
    std::cout << "size_t: \t"
              << "所占字节数：" << sizeof(size_t);
    std::cout << "\t最大值：" << (std::numeric_limits<size_t>::max)();
    std::cout << "\t最小值：" << (std::numeric_limits<size_t>::min)() << std::endl;
    std::cout << "string: \t"
              << "所占字节数：" << sizeof(std::string) << std::endl;
    std::cout << "\t最大值：" << (std::numeric_limits<string>::max)();

    std::cout << "\t最小值：" << (std::numeric_limits<string>::min)() << std::endl;
    std::cout << "type: \t\t"
              << "************size**************" << std::endl;
    Eigen::Matrix<double, -1, -1> a_testeigen;

    std::cout << "a_testeigen: " << a_testeigen.size() << std::endl;
    int a = 15;
    int b = 30;
    pointFunTest(&a, &b);
    std::cout << "a: " << a << " b: " << b << std::endl;

    cout << "字符串--------------------------------------------" << std::endl;
    char site[7] = {'R', 'U', 'N', 'O', 'O', 'B', '\0'};
    std::string site_1 = "runoob";
    std::cout << "字符串测试: ";
    std::cout << site << "size:" << strlen(site) << std::endl;
    std::cout << "string: " << site_1 << "size:" << site_1.size() << std::endl;
    std::cout << "site地址：" << &site << std::endl;
    std::cout << "site1地址：" << &site_1 << std::endl;
    std::cout << "a   地址：" << &a << std::endl;

    cout << "指针--------------------------------------------" << std::endl;
    int *ptr_test;
    int **ptr_ptr_test;
    int *ptr_test_a = &b;
    ptr_test = NULL;
    ptr_ptr_test = &ptr_test_a;
    std::cout << "ptr_test=: " << ptr_test_a << std::endl;
    std::cout << "*ptr_test=: " << *ptr_test_a << std::endl;
    std::cout << "*ptr_ptr_test=: " << *ptr_ptr_test << std::endl;
    std::cout << "**ptr_ptr_test=: " << **ptr_ptr_test << std::endl;

    std::cout << "空指针地址=: " << ptr_test << std::endl;

    // 使用指针代替数组，因为变量指针可以递增，而数组不能递增，因为数组是一个常量指针面的程序递增变量指针，
    // 以便顺序访问数组中的每一个元素
    cout << "指针运算--------------------------------------------" << std::endl;
    int var[5] = {10, 100, 200, 1243, 12314};
    int *ptr;
    // 指针中的数组地址
    // var是一个指向数组开头的常量
    ptr = var;
    for (int i = 0; i < 5; i++)
    {
        cout << "Address of var[" << i << "] = ";
        cout << ptr << endl;

        cout << "Value of var[" << i << "] = ";
        cout << *ptr << endl;
        // 移动到下一个位置
        ptr++;
    }

    cout << "指针与数组--------------------------------" << std::endl;
    const char *names[4] = {
        "Zara Ali",
        "Hina Ali",
        "Nuha Ali",
        "Sara Ali",
    };

    for (int i = 0; i < 4; i++)
    {
        cout << " --- names[i]              = " << names[i] << endl;
        cout << " --- *names[i]             = " << *names[i] << endl;
        cout << endl;
        cout << " --- (*names[i] + 1)       = " << (*names[i] + 1) << endl;
        cout << " --- (char)(*names[i] + 1) = " << (char)(*names[i] + 1) << endl;
        cout << " ------------------------------------ " << endl
             << endl
             << endl
             << endl;
    }

    cout << "日期和时间------------------------------------------" << std::endl;
    time_t now = time(0);

    cout << "1970 到目前经过秒数:" << now << endl;

    tm *ltm = localtime(&now);
    cout << "当前时间为：" << std::endl;
    // 输出 tm 结构的各个组成部分
    cout << 1900 + ltm->tm_year << "年";
    cout << 1 + ltm->tm_mon << "月";
    cout << ltm->tm_mday << "日";
    cout << ltm->tm_hour << ":";
    cout << ltm->tm_min << ":";
    cout << ltm->tm_sec << endl;

    float x = 0.00403826;
    float y = 0.4123;
    float z = 0.910945;
    float n1[3] = {0, 1, 0};
    float normal_vec[3] = {x, y, z};
    float q1[3];
    Eigen::Vector3f q;
    Eigen::Vector3f y1;
    // Eigen::Vector3f  y_axis(0,1,0);
    // Eigen::Vector3f nor_v(0.00403826,0.4123,0.910945);
    // Eigen::Vector3f rotation_axis = y_axis.cross(-nor_vect).normalized();

    cal_outer_product(normal_vec, n1, q1);
    y1 << n1[0], n1[1], n1[2];
    q << normal_vec[0], normal_vec[1], normal_vec[2];
    std::cout << "q1 计算：" << q1[0] << " " << q1[1] << " " << q1[2] << std::endl;
    Eigen::Vector3f rotation_axis = q.cross(y1);

    std::cout << "rotation_axis:\n"
              << rotation_axis << std::endl;
    std::vector<int> a_test = {1, 2, 3, 4, 5, 6};
    cout << "a_test" << a_test;
    char data[100];

    // 基本的线性方程组求解 Solving linear least squares systems
    Eigen::Matrix<float, 10, 2> mat_A;
    mat_A << 1, 1,
        2, 1,
        3, 1,
        4, 1,
        5, 1,
        6, 1,
        7, 1,
        8, 1,
        9, 1,
        10, 1;
    std::cout << "matrix A: " << mat_A << std::endl;

    Eigen::VectorXf vet_b = Eigen::VectorXf::Random(10);
    // Eigen::Matrix<float,10,1> vet_b;
    vet_b << 6.9918, 14.2987, 16.2019, 22.4263, 25.6191, 33.2563, 35.7755, 42.0298, 47.9954, 53.9545;
    std::cout << "vet_b " << vet_b << std::endl;
    // 最小二乘求解 BDCSVD
    std::cout << "The least-squares solution is:\n"
              << mat_A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(vet_b) << endl;

    // using the QR decomposition
    std::cout << "The solution using the QR decomposition is:\n"
              << mat_A.colPivHouseholderQr().solve(vet_b).transpose() << std::endl;

    // using normal equations
    std::cout << "The solution using normal equations is:\n"
              << (mat_A.transpose() * mat_A).ldlt().solve(mat_A.transpose() * vet_b).transpose()
              << std::endl;

    int num = 10;
    auto fun = [&]
    {   num=100;
        std::cout << num << std::endl;
        std::string a = "asdad";
        return a;
    };
    fun();

    Eigen::Vector3f skew_a(2.1,2,2.2);
    
    return 0;
}
