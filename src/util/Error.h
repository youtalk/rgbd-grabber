/**
 * @file Error.h
 * @author Yutaka Kondo <yutaka.kondo@kawadarobot.co.jp>
 * @date Dec 16, 2013
 */

#ifndef ERROR_H_
#define ERROR_H_

namespace krc {

class UnsupportedException: public std::domain_error {
public:
    UnsupportedException(const std::string& cause) :
            std::domain_error(cause + ": unsupported") {}
};

}

#endif /* ERROR_H_ */
