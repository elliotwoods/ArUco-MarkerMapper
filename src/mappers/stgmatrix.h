#ifndef STGMATRIX_H
#define STGMATRIX_H
#include <vector>
#include <iostream>
namespace aruco_mm{
/**
 * A   Matrix based to store data
 */
template<typename T>
class   StgMatrix  {

    std::vector<T> data;
    uint32_t _rows,_cols;

public:
    /**Returns the number of rows
      */
    inline size_t rows()const {
        return _rows;
    }
    /**Returns the number of cols
      */
    inline size_t cols()const {
        return _cols;
    }

    /**
     */
    StgMatrix( ) {
        _rows=_cols=0;
    }
    /**
     */
    StgMatrix(size_t nr,size_t nc ) {
        _rows=_cols=0;
        resize(nr,nc);
    }

    /**
     */
    void resize(size_t nr,size_t nc) {
        _rows=nr;
        _cols=nc;
        data.clear();
        data.resize(_rows*_cols);
    }

    /**
     */
    inline T & operator()(size_t idx) {
        return data.at(idx);
    }
    /**
     */
    const T & operator()(size_t idx)const {
        return data.at(idx);
    }

    /**
     */
    inline T & operator()(size_t r,size_t c) {
        assert(r<_rows && c<_cols);
        return data.at(get1dIdx(r,c));
    }
    /**
     */
    inline const T & operator()(size_t r,size_t c) const {
        assert(r<_rows && c<_cols);
        return data.at(get1dIdx(r,c));
    }


    /**
     */
    inline size_t size()const {
        return data.size();
    }


    void toStream(std::ostream &str){
        str.write((char*)&_rows,sizeof(uint32_t));
        str.write((char*)&_cols,sizeof(uint32_t));
        for(auto &e:data) e.toStream(str);
    }

    void fromStream(std::istream &str){
        str.read((char*)&_rows,sizeof(uint32_t));
        str.read((char*)&_cols,sizeof(uint32_t));
        resize(_rows,_cols);
        for(auto &e:data) e.fromStream(str);

    }

    void clear(){
        data.clear();
        _rows=_cols=-1;
    }

private:
    size_t get1dIdx(int r, int c)const {
        return r*_cols+c;
    }
};
}
#endif // STGMATRIX_H
